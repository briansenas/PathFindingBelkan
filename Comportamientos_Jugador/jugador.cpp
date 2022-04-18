#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <set>
#include <stack>
#include <queue>
#include <algorithm>
#include <vector>
#include <set>
#include <queue>
#include <limits>

#define CHARGE true //Por si queremos que tenga en cuenta cargar la bateria
#define NOTIMELIMIT false //Por si podríamos quitar el temporizador de think.
#define NOMOVEMENTLIMIT false
#define AVOIDVILLAGER false


void Cost(unsigned char casilla, nodoCoste& obj, const estado destino, double FactorOptimizacion);
// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a realizar.
// Para ver los distintos sensores mirar fichero "comportamiento.hpp"
Action ComportamientoJugador::think(Sensores sensores) {
    if(!Parameters){
        Parameters = true;
        THOUGHT = false;
        charging = false;
        aldeano = false;
        cout << "[MAPSIZE]: " <<  mapaResultado.size() << endl;
        //For faster calculus, además utiliza menos movimientos.
       // FP = (((double)mapaResultado.size())/100+0.05)/2; //Este valor se me ocurrió en un sueño.
        FP = 0; //Habría que probar valores para encontrar un óptimo global, pero me da pereza.
        MINBATTERY = (((double)mapaResultado.size())/10) * 100 + 300; //Valores que se me ocurrió en un sueño.
        THRESHOLD = ((((double)mapaResultado.size())) * 10)/2 + mapaResultado.size(); //Same ^^^
        MAXCHARGE = 3000; //Because yes
        total_cost = 0;
        ITER = 3000; //Is this cheating??... xd
        diff = 0;
        exp_rate = 1;
        cout << "[MINBATTERY]: " << MINBATTERY << endl;
        cout << "[THRESHOLD]: " << THRESHOLD << endl;
        cout << "[MAXCHARGE]: " << MAXCHARGE << endl;
        cout << "[FP]: " << FP << endl;
    }
    Action accion = actIDLE;
    if(!NOMOVEMENTLIMIT) {
        if (MINBATTERY > THRESHOLD) {
            MINBATTERY -= 0.001;
        } else {
            MINBATTERY = THRESHOLD;
        }
        ITER--;
        if (ITER <= 0)
            ITER = 0;
        MAXCHARGE = MINBATTERY + THRESHOLD + ITER;
        if (MAXCHARGE > 2980)
            MAXCHARGE = 2980;
        exp_rate += 0.0002;
        if(exp_rate >= 3){
            exp_rate = 3;
        }
    }

    actual.fila        = sensores.posF;
    actual.columna     = sensores.posC;
    actual.orientacion = sensores.sentido;
    if(!hay_plan  and objetivos.empty()) {
        objetivos.clear();
        index_bateria = 0;
        index_cercano = 0;
        for (int i = 0; i < sensores.num_destinos; i++) {
            estado aux;
            aux.fila = sensores.destino[2 * i];
            aux.columna = sensores.destino[2 * i + 1];

            objetivos.push_back(aux);
        }
    }

    if(sensores.nivel==4){
        if(THOUGHT) {
            diff = sensores.tiempo - OLD;
            THOUGHT = false;
            cout << "[DIFF]: " << diff << endl;
        }
        else
            OLD = sensores.tiempo;

        estado temp = actual;
        if(aldeano and AVOIDVILLAGER){
           aldeano = false;
        }else
            rellenaMapa(actual,sensores.terreno);
        bool charger = NearByCharger();
        EsObjeto(actual);

        if(plan.front()==actFORWARD) {
            if(HayObstaculoDelante(temp)) {
                hay_plan = false;
                busca_objeto = false;
            }
            unsigned char verify = mapaResultado[temp.fila][temp.columna];
            //Cómo al final he optimizado el cálculo de los caminos, diff ya no sirve para este valor, pero lo dejo por si acaso.
            if((diff<40 or NOTIMELIMIT) and !charging){ //Si ha tardado demasiado en calcular el camino no renta volver a intentarlo por coste.
                if (verify == 'B' and actual.objeto != 'D' and !alrededorCostoso(temp)) {
                    cout << "[QUITE EXPENSIVE INNIT?]" << endl;
                    hay_plan = false;
                    busca_objeto = false;
                } else if (verify == 'A' and actual.objeto != 'K' and !alrededorCostoso(temp)) {
                    cout << "[QUITE EXPENSIVE INNIT?]" << endl;
                    hay_plan = false;
                    busca_objeto = false;
                }
            }
            //Miramos una casilla hacia alante para evitar entrar en situaciones peligrosas para la batería.
            double next = LookAhead(actual);
            if(sensores.bateria-next<=MINBATTERY and !baterias.empty() and !charging){
                if(verify=='A' and actual.objeto!='K' or st_bat>=sensores.bateria){
                    cout << "[BETTER GET MY JUICE]:" << next << endl;
                    hay_plan = false;
                    busca_bateria = true;
                    busca_objeto = false;
                }else if(verify=='B' and actual.objeto!='D' or st_bat>=sensores.bateria){
                    cout << "[BETTER GET MY JUICE]: " << next << endl;
                    hay_plan = false;
                    busca_bateria = true;
                    busca_objeto = false;
                }
            }
        }

        //Verificamos que hemos encontrado un objetivo.
        if (!objetivos.empty() and actual.fila == objetivos[index_cercano].fila and actual.columna == objetivos[index_cercano].columna) {
            cout << "[OBJECTIVE FOUND]" << objetivos.size() << endl;
            //FP = (((double)mapaResultado.size())/100+0.05)/2; //Restaurar FP en caso de modificación.
            hay_plan = false;
            busca_objeto = false;
            if(!baterias.empty() and sensores.bateria-LookAhead(actual)<=MINBATTERY and DistBat(actual)<=18)
                busca_bateria = true;

            if (objetivos.size() > 1) {
                objetivos.erase(objetivos.begin() + index_cercano);
            } else {
                objetivos.clear();
            }
        }
        //Mirar batería mientras vamos a por un objetivo.
        if(!busca_bateria and sensores.bateria<=MINBATTERY) {
            if(!objetivos.empty()){
                if(DistBat(actual)<=DistBat(objetivos[index_cercano]) and (plan.size()>10 and alrededorCostoso(actual))) {
                    cout << "[RUNNING OUT OF BATTERY]" << endl;
                    hay_plan = false;
                    busca_objeto = false;
                    if(!baterias.empty())
                        busca_bateria = true;
                }
            }else{
                cout << "[RUNNING OUT OF BATTERY]" << endl;
                hay_plan = false;
                busca_objeto = false;
                if(!baterias.empty())
                    busca_bateria = true;
            }
        }
        else if(busca_bateria and sensores.bateria>=MAXCHARGE){ //verificar terminar de cargar
            //FP = (((double)mapaResultado.size())/100+0.05)/2;
            //cout << "[FP]: " << FP << endl;
            hay_plan = false;
            busca_objeto = false;
            busca_bateria = false;
        }

        //Buscar objetivo más cercano. (utilizamos la distancia euclidiana para este cálculo pero lineal para A*).
        double distance = 0;
        int index = index_cercano;
        if(!objetivos.empty() and !busca_objeto and !busca_bateria) {
            min_distance = numeric_limits<double>::max();
            for (int i = 0; i < objetivos.size(); i++) {
                int df = abs(actual.fila - objetivos[i].fila);
                int dc = abs(actual.columna - objetivos[i].columna);
                //if (df == 0) df = 1;
                //if (dc == 0) dc = 1;
                df = df * df; dc = dc * dc;
                distance = df + dc ;
                if (distance <= min_distance)  {
                    min_distance = distance;
                    index = i;
                }
            }
        }
        if (index != index_cercano) { //Nos hemos acercado a otro objetivo
            hay_plan = false;
            index_cercano = index;
        }
    }
    //Calcular nuevo plan si no hay uno ni tampoco buscamos bateria.
    if(!hay_plan and !objetivos.empty() and !busca_bateria) {
        cout << "[RECALCULATING...]" << endl;
        cout << "[CHECKING LOCATION] "  << index_cercano << " - " << objetivos.size() << endl;
        cout << "[CALCULATED MIN]" << " [R:" << min_distance << "]" << endl;
        plan.clear();
        hay_plan = pathFinding(sensores, actual, objetivos, plan);
        THOUGHT = true; //hemos pensado, toca calcualr el tiempo que hemos tardado.
        goal = objetivos.front();
        //PintaPlan(plan);
        // ver el plan en el mapa
        VisualizaPlan(actual, plan);
    }

    Action sigAccion = actIDLE;
    if(!sensores.colision) {
        if(CHARGE) { //Si no quiero que se quede a cargar bateria desactivar variable
            if (mapaResultado[actual.fila][actual.columna] == 'X') {
                baterias.insert(actual);
                charging = true; //Estadísticamente renta más siempre estar cargado, así que why not.
                if (sensores.bateria >= MAXCHARGE) {
                    //FP = (((double)mapaResultado.size())/100+0.05)/2;
                    //cout << "[FP]: " << FP << endl;
                    busca_bateria = false;
                    charging = false;
                }else if(!charging){
                    cout << "[MINBATTERY]: " << MINBATTERY << endl;
                    cout << "[BATTERY FOUND]" << endl;
                    cout << "[TOTAL]: " << sensores.bateria << endl;
                }
            } else if ((!hay_plan and busca_bateria and !baterias.empty()) or
                    (!busca_bateria and !baterias.empty() and sensores.bateria-LookAhead(actual)<=MINBATTERY+LookAhead(actual) and DistBat(actual) <= 50)){
                busca_bateria = true;
                cout << "[SEARCHING FOR MY CHARGER]" << endl;
                //cout << "[FP]: " << FP << endl;
                DistBat(actual);
                cout << "[DISTANCE]: " << min_distance << endl;
                vector <estado> nuevo;
                auto res = baterias.begin();
                for (int i = 0; i < index_bateria and res != baterias.end(); i++) {
                    if(res!=baterias.end());
                        res++;
                }
                nuevo.push_back(*res);
                int i = 0;
                do {
                    if(i>5) {
                        cout << "[SORRY, COULDN'T DO IT..]" << endl;
                        cout << "[IT'S THE END :( ... bye..]" << endl;
                        break;
                    } //Only if using A*;
                    plan.clear();
                    hay_plan = pathFinding(sensores, actual, nuevo, plan);
                    /*FP -= 0.15;
                    if(FP<=0)
                        FP = 0;*/ //Only if using A*;
                    if(st_bat>=sensores.bateria)
                        cout << "[TOO EXPENSIVE, TRYING AGAIN...]" << endl;
                    i++;
                }while(st_bat >= sensores.bateria); //Not worth a path that kills us.
                THOUGHT = true;
                //PintaPlan(plan);
                // ver el plan en el mapa
                VisualizaPlan(actual, plan);
            }
        }

        if ((hay_plan or busca_bateria) and plan.size() > 0 and !charging) {
            sigAccion = plan.front();
            plan.erase(plan.begin());
        }
        else if(plan.empty() and !objetivos.empty()){
            hay_plan = false;
        }
    }
    else{
        //Pensé que los aldeanos podrían ocasionar situaciones complicadas, pero en la mayoria de las veces no.
        if(AVOIDVILLAGER and sensores.nivel==4){ //Así que para las condiciones del juego es mejor dejarlo en FALSE
            hay_plan = false;
            busca_objeto = false;
            estado temp = actual;
            if(!HayObstaculoDelante(temp) and !(temp.fila==objetivos[index_cercano].fila
                                                and temp.columna==objetivos[index_cercano].columna) ) {
                if (temp.fila >= 0 and temp.fila < mapaResultado.size()
                    and temp.columna >= 0 and temp.columna < mapaResultado[temp.fila].size()) {
                    cout << "[VILLAGER IN THE WAY]" << endl;
                    mapaResultado[temp.fila][temp.columna] = 'P';
                    aldeano = true;
                }
            }
        }else
            sigAccion = last_action;
    }
    last_action = sigAccion;
    return sigAccion;
}
double ComportamientoJugador::LookAhead(estado st){
    int fil=st.fila, col=st.columna;
    double next_bat = 0;
    // calculo cual es la casilla de delante del agente
    for(int i=0;i<1;i++) {
        switch (st.orientacion) {
            case 0: fil--; break;
            case 1: col++; break;
            case 2: fil++; break;
            case 3: col--; break;
        }
        if (fil >= 0 and fil < mapaResultado.size() and col >= 0 and col < mapaResultado[fil].size()) {
            unsigned char var = mapaResultado[fil][col];
            nodoCoste obj;
            obj.st = st;
            Cost(var, obj, st, 0);
            next_bat += obj.bat;
            st_bat += next_bat;
        }
    }
    return next_bat;
}

bool ComportamientoJugador::alrededorCostoso(estado temp){
    bool cost = true;
    for(int i=-1;i<=1;i++) {
        for (int j = -1; j <=1; j++) {
            if (temp.fila + i >= 0 and temp.fila + i < mapaResultado.size() and
                temp.columna + j >= 0 and temp.columna + j < mapaResultado[temp.fila + i].size()) {
                unsigned char var = mapaResultado[temp.fila + i][temp.columna + j];
                if (!EsObstaculo(var)) { //NOT 'P' not 'M'
                    if (var != 'B' and var != 'A') {
                        cost = false;
                        break;
                    }
                }
            }
        }
    }
    return cost;
}
bool ComportamientoJugador::NearByCharger(){
    bool found = false;
    for(int i=-2;i<=2;i++) {
        for (int j = -2; j <= 2; j++) {
            if (actual.fila + i >= 0 and actual.fila + i < mapaResultado.size() and
                actual.columna + j >= 0 and actual.columna + j < mapaResultado[actual.fila + i].size()) {
                unsigned char temp = mapaResultado[actual.fila + i][actual.columna + j];
                if (!EsObstaculo(temp)) {
                    estado aux = actual;
                    if (!EsObjeto(aux)) {
                        if (temp == 'X') {
                            estado res;
                            res.fila = actual.fila+i;
                            res.columna = actual.columna + j;
                            baterias.insert(res);
                            found = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    return found;
}

double ComportamientoJugador::DistBat(estado temp){
    auto bat = baterias.begin();
    auto res = bat;
    min_distance = numeric_limits<double>::max();
    double distance = 0;
    int cont = -1;
    for(;bat!=baterias.end();++bat){
        cont++;
        int df =  abs(temp.fila - bat->fila);
        int dc =  abs(temp.columna - bat->columna);
        //if(df==0) df = 1;
        //if(dc==0) dc = 1;
        df = df*df; dc = dc * dc;
        distance = df + dc;
        if (distance <= min_distance) {
            index_bateria = cont;
            min_distance = distance;
            res = bat;
        }
    }
    return min_distance;
}

// Llama al algoritmo de busqueda que se usara en cada comportamiento del agente
// Level representa el comportamiento en el que fue iniciado el agente.
bool ComportamientoJugador::pathFinding (Sensores sensores , const estado &origen, const vector<estado> &destino, list<Action> &plan){
    st_bat = 0;
    double totalCost = 0;
    int level = sensores.nivel;
    int distance=0;
    estado un_objetivo;
    switch (level){
        case 0: cout << "Demo\n";
            un_objetivo = objetivos.front();
            plan.clear();
            cout << "fila: " << un_objetivo.fila << " col:" << un_objetivo.columna << endl;
            return pathFinding_Profundidad(origen,un_objetivo,plan);
            break;

        case 1: cout << "Optimo numero de acciones\n";
            // Incluir aqui la llamada al busqueda en anchura
            un_objetivo = objetivos.front();
            plan.clear();
            cout << "fila: " << un_objetivo.fila << " col:" << un_objetivo.columna << endl;
            return pathFinding_Anchura(origen,un_objetivo,plan);
            cout << "[TOTAL COST]: " << totalCost << endl;
            break;

        case 2: cout << "Optimo en coste 1 Objetivo\n";
            // Incluir aqui la llamada al busqueda de costo uniforme
            un_objetivo = objetivos.front();
            plan.clear();
            cout << "fila: " << un_objetivo.fila << " col:" << un_objetivo.columna << endl;
            return pathFinding_CosteUniforme(origen,un_objetivo,plan,0,totalCost);
            break;

        case 3: cout << "Optimo en coste 3 Objetivos\n";
            // Incluir aqui la llamada al algoritmo de busqueda para 3 objetivos

            /*En este caso para NOBjetivos(..) El factor de optimización es necesario que sea elevado
            Sea suficiente para que incluso las menores distancias pesen lo suficiente como para
            que sea rentable ir por un terreno costoso como podría ser el agua.*/
            return pathFinding_NObjetivos(origen,objetivos,plan,0,totalCost); //TimeComplexity(b^d)

            /*Aqui como reutilizo CosteUniforme, sigue prevaleciendo un valor pequeño para la distancia */
            //return pathFinding_Krusgal(origen,objetivos,plan,0,totalCost); //TimeComplexity+++ O(n!*b^d);

            break;

        case 4:
            // Incluir aqui la llamada al algoritmo de busqueda usado en el nivel 2
            if(!busca_bateria) {
                un_objetivo = objetivos[index_cercano];
                return pathFinding_CosteUniforme(origen,un_objetivo,plan,FP,totalCost);
            } else {
                un_objetivo = destino[0];
                return pathFinding_CosteUniforme(origen,un_objetivo,plan,FP,totalCost);
            }
            //return pathFinding_NObjetivos(origen,objetivos,plan,3,totalCost); //TimeComplexity(b^d)
            break;
    }
    return false;
}


//---------------------- Funciones de las busquedas ---------------------------


// Dado el codigo en caracter de una casilla del mapa dice si se puede
// pasar por ella sin riegos de morir o chocar.
bool ComportamientoJugador::EsObstaculo(unsigned char casilla){
    if (casilla=='P' or casilla=='M')
        return true;
    else
        return false;
}


// Comprueba si la casilla que hay delante es un obstaculo. Si es un
// obstaculo devuelve true. Si no es un obstaculo, devuelve false y
// modifica st con la posición de la casilla del avance.
bool ComportamientoJugador::HayObstaculoDelante(estado &st){
    int fil=st.fila, col=st.columna;
    // calculo cual es la casilla de delante del agente
    switch (st.orientacion) {
        case 0: fil--; break;
        case 1: col++; break;
        case 2: fil++; break;
        case 3: col--; break;
    }
    // Compruebo que no me salgo fuera del rango del mapa
    if (fil<0 or fil>=mapaResultado.size()) return true;
    if (col<0 or col>=mapaResultado[0].size()) return true;

    // Miro si en esa casilla hay un obstaculo infranqueable
    if (!EsObstaculo(mapaResultado[fil][col])){
        // No hay obstaculo, actualizo el parametro st poniendo la casilla de delante.
        st.fila = fil;
        st.columna = col;
        return false;
    }
    else{
        return true;
    }
}

struct ComparaEstados{
    bool operator()(const estado &a, const estado &n) const{
        bool diff = false;
        if(!a.dests.empty() and !n.dests.empty()) {
            for (int i = 0; i < a.dests.size() - 1 and i < n.dests.size() - 1; ++i) {
                if (a.dests[i] > n.dests[i]) {
                    diff = true;
                    break;
                }

                if (a.dests[i] == n.dests[i] and a.dests[i + 1] > n.dests[i + 1]) {
                    diff = true;
                    break;
                }
            } //For some reason this is not working...
        }

        if(!a.dests.empty() and !n.dests.empty()) {
            if ((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna) or
                (a.fila == n.fila and a.columna == n.columna and a.orientacion > n.orientacion) or
                (a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.objeto > n.objeto) or
                (a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.objeto == n.objeto
                and a.dests[0] > n.dests[0]) or
                (a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.objeto == n.objeto
                 and a.dests[0] == n.dests[0] and a.dests[1] > n.dests[1]) or
                (a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.objeto == n.objeto
                 and a.dests[0] == n.dests[0] and a.dests[1] == n.dests[1] and a.dests[2] > n.dests[2]))
                //(a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.objeto == n.objeto and diff))
                return true;
            else
                return false;
        }
        else{
            if ((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna) or
                (a.fila == n.fila and a.columna == n.columna and a.orientacion > n.orientacion) or
                (a.fila == n.fila and a.columna == n.columna and a.orientacion == n.orientacion and a.objeto > n.objeto))
                return true;
            else
                return false;
        }
    }
};

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------- RESOLUCIÓN DEL NIVEL 1 ----------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

//---------------------- Implementación de la busqueda en anchura ---------------------------

/**
 * Para realizar este ejercicio simplemente hemos tenido que realizar un cambio en la estructura de datos que utilizaba
 * nuestro código en profundidad. Cambiar que abiertos fuera de tipo "stack" a "queue" para que tenga que realizar una
 * búsqueda sobre todos los hijos generados por un nodo antes de avanzar un nivel más de profunidad.
 * @param origen dónde estamos
 * @param destino dónde vamos
 * @param plan camino a seguir
 * @return si lo hemos encontrado o no.
 */
bool ComportamientoJugador::pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan) {
    //Borro la lista
    cout << "Calculando plan\n";
    plan.clear();
    set<estado,ComparaEstados> Cerrados; // Lista de Cerrados
    queue<nodo> Abiertos;								 // Lista de Abiertos

    nodo current;
    current.st = origen;
    current.secuencia.empty();

    Abiertos.push(current);
    while (!Abiertos.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){
        Abiertos.pop();
        Cerrados.insert(current.st);

        // Generar descendiente de girar a la derecha
        nodo hijoTurnR = current;
        hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
        if (Cerrados.find(hijoTurnR.st) == Cerrados.end()){
            hijoTurnR.secuencia.push_back(actTURN_R);
            Abiertos.push(hijoTurnR);
        }

        // Generar descendiente de girar a la izquierda
        nodo hijoTurnL = current;
        hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
        if (Cerrados.find(hijoTurnL.st) == Cerrados.end()){
            hijoTurnL.secuencia.push_back(actTURN_L);
            Abiertos.push(hijoTurnL);
        }

        // Generar descendiente de avanzar
        nodo hijoForward = current;
        if (!HayObstaculoDelante(hijoForward.st)){
            if (Cerrados.find(hijoForward.st) == Cerrados.end()){
                hijoForward.secuencia.push_back(actFORWARD);
                if(hijoForward.st.fila==destino.fila and hijoForward.st.columna == destino.columna) {
                    current = hijoForward;
                    break;
                }
                Abiertos.push(hijoForward);
            }
        }

        // Tomo el siguiente valor de la Abiertos
        if (!Abiertos.empty()){
            current = Abiertos.front();
        }
    }

    cout << "Terminada la busqueda\n";

    if (current.st.fila == destino.fila and current.st.columna == destino.columna){
        cout << "Cargando el plan\n";
        plan = current.secuencia;
        cout << "Longitud del plan: " << plan.size() << endl;

        return true;
    }
    else {
        cout << "No encontrado plan\n";
    }


    return false;
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------- RESOLUCIÓN DEL NIVEL 2 ----------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------


struct ComparaCostes{
    bool operator()(const nodoCoste &a, const nodoCoste &n) const{
        if(a.total_cost == n.total_cost and (a.st.objeto!='0' and n.st.objeto=='0'))
            return true;
        if(a.total_cost == n.total_cost and (a.st.objeto=='0' and n.st.objeto!='0'))
            return false;
        else if(a.total_cost == n.total_cost)
            return a.cost<=n.cost;
        else if(a.total_cost < n.total_cost)
            return true;
        else
            return false;
    }
};

/**
 * Permite introducir un coste a un nodo según el tipo de terreno y movimiento a realizar e incluso incluir la distancia
 * al objetivo según un Factor para tener una función f(n)=h(n)+g(n) -> A* en vez de CosteUniforme.
 * @param casilla tipo de terreno que estamos
 * @param obj nodo a almacenar el coste
 * @param destino dónde tenemos que ir
 * @param FactorOptimizacion cantidad en la que influye la distancia al objetivo
 */
void ComportamientoJugador::Cost(unsigned char casilla, nodoCoste& obj, const estado destino, double FactorOptimizacion=0){
    if(casilla=='A') {
        if(obj.secuencia.back()==actFORWARD) {
            if(obj.st.objeto=='K')
                obj.cost=10;
            else
                obj.cost=200;
        }
        else {
            if(obj.st.objeto=='K')
                obj.cost=5;
            else
                obj.cost=500;
        }
    }
    else if(casilla=='B') {
        if(obj.secuencia.back()==actFORWARD) {
            if(obj.st.objeto=='D')
                obj.cost=15;
            else
                obj.cost=100;
        }
        else {
            if(obj.st.objeto=='D')
                obj.cost=1;
            else
                obj.cost=3;
        }
    }
    else if(casilla=='T')
        obj.cost=2;
    else
        obj.cost=1;

    if(casilla=='?')
        obj.cost = exp_rate;

    obj.bat += obj.cost; //Apenas coste bateria en caso de estar utilizando A*

    if(obj.secuencia.back()==actFORWARD){
        int dFila = abs(destino.fila - obj.st.fila);
        int dColumna = abs(destino.columna - obj.st.columna);
        obj.total_cost += (dFila+dColumna)*FactorOptimizacion;
    }

    if(casilla=='X')
        obj.cost=0;

    obj.cost += 1; //Tener en cuenta iteracciones

    obj.total_cost+=obj.cost;
}

//---------------------- Implementación de la búsqueda de CosteUniforme o A* [NIVEL 2] ---------------------------

bool  ComportamientoJugador::EsObjeto(estado &obj){
    if(mapaResultado[obj.fila][obj.columna]=='D') {
        obj.objeto = 'D';
        busca_objeto = true; //Saber que hemos elegido un camino para el cual necesitamos un objeto
        return true;
    }
    else if(mapaResultado[obj.fila][obj.columna]=='K') {
        obj.objeto = 'K';
        busca_objeto = true; //Saber que hemos elegido un camino para el cual necesitamos un objeto
        return true;
    }
    return false;
}

/**
 * Para resolver el nivel 2 basta con utilizar una estructura de datos que ordene los nodos según los coste de los
 * movimientos debido al terreno en que me encuentro. (He ido por multiset en vez de priority queue por razones
 * personales de preferencia, sobre todo en la creacion de la comparacion de costes ).
 * PD: La única razón por la que <<estado &destino>> no es constante es por uno de los métodos de resolución del nivel 3
 * En concreto, no por el A* sino por el algoritmo de Krusgal (Que no se usa al final debido a su complejidad temporal).
 * @param origen dónde estamos
 * @param destino dónde vamos
 * @param plan camino que utilizamos
 * @param FactorOptimizacion factor de influyencia de la distancia al destino
 * @param totalCost coste total del camino
 * @return si hemos encontrado un camino
 */
bool ComportamientoJugador::pathFinding_CosteUniforme(const estado &origen,  estado &destino, list<Action> &plan,double FactorOptimizacion, double& totalCost) {
    //Borro la lista
    cout << "Calculando plan\n";
    //plan.clear();
    set<estado,ComparaEstados> Cerrados; // Lista de Cerrados
    multiset<nodoCoste, ComparaCostes> Abiertos;								 // Lista de Abiertos

    nodoCoste current;
    current.st = origen;
    current.secuencia.empty();

    Abiertos.insert(current);

    int fil=current.st.fila, col=current.st.columna;
    unsigned char casilla = mapaResultado[fil][col];
    while (!Abiertos.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){
        auto it = Abiertos.begin();
        while(Cerrados.find(current.st)!=Cerrados.end()) {
            Abiertos.erase(it);
            it = Abiertos.begin();
            current = *it;
            if(Abiertos.empty())
                break;
        }

        Cerrados.insert(current.st);

        //Miramos que casilla estamos.
        fil = current.st.fila;
        col = current.st.columna;
        casilla = mapaResultado[fil][col];
        current.casilla = casilla;

        // Generar descendiente de girar a la derecha
        nodoCoste hijoTurnR = current;
        hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion + 1) % 4;
        if (Cerrados.find(hijoTurnR.st) == Cerrados.end()) {
            hijoTurnR.secuencia.push_back(actTURN_R);
            Cost(casilla, hijoTurnR, destino, 0);
            Abiertos.insert(hijoTurnR);
        }

        // Generar descendiente de girar a la izquierda
        nodoCoste hijoTurnL = current;
        hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion + 3) % 4;
        if (Cerrados.find(hijoTurnL.st) == Cerrados.end()) {
            hijoTurnL.secuencia.push_back(actTURN_L);
            Cost(casilla, hijoTurnL, destino, 0);
            Abiertos.insert(hijoTurnL);
        }

        // Generar descendiente de avanzar
        nodoCoste hijoForward = current;
        if (!HayObstaculoDelante(hijoForward.st)) {
            if (Cerrados.find(hijoForward.st) == Cerrados.end()) {
                fil = hijoForward.st.fila;
                col = hijoForward.st.columna;
                casilla = mapaResultado[fil][col];
                EsObjeto(hijoForward.st);
                hijoForward.secuencia.push_back(actFORWARD);
                Cost(casilla, hijoForward, destino, FactorOptimizacion);
                Abiertos.insert(hijoForward);
            }
        }
        // Tomo el siguiente valor de la Abiertos
        if (!Abiertos.empty()) {
            it = Abiertos.begin();
            current = *it;
        }
    }

    //cout << "Terminada la busqueda\n";

    if (current.st.fila == destino.fila and current.st.columna == destino.columna){
        destino = current.st;
        //   cout << "Cargando el plan\n";
        cout << "[OVERALL COST]: " << current.bat << endl;
        totalCost = current.total_cost;
        st_bat = current.bat;
        total_cost = current.total_cost;
        plan.insert(plan.end(),current.secuencia.begin(),current.secuencia.end());
        cout << "Longitud del plan: " << plan.size() << endl;

        return true;
    }
    else {
        cout << "No encontrado plan\n";
    }


    return false;

}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//------------------------------------------- RESOLUCIÓN DEL NIVEL 3 ---------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------


/**
 * Añado el coste de movimiento según el tipo de este y el tipo de terreno. A la vez influye el mínimo de las
 * distancias a los objetivos que no han sido encontrado por una Factor "FactorOptimizacion".
 * @param casilla dónde me encuentro
 * @param obj nodo dónde almacenar el coste
 * @param destino vector de objetivos
 * @param FactorOptimizacion factor de influencia de la distancia
 */
void Cost2(unsigned char casilla, nodoCoste& obj, const vector<estado> destino,double FactorOptimizacion=0){
    if(casilla=='A') {
        if(obj.secuencia.back()==actFORWARD) {
            if(obj.st.objeto=='K')
                obj.cost=10;
            else
                obj.cost=200;
        }
        else {
            if(obj.st.objeto=='K')
                obj.cost=5;
            else
                obj.cost=500;
        }
    }
    else if(casilla=='B') {
        if(obj.secuencia.back()==actFORWARD) {
            if(obj.st.objeto=='D')
                obj.cost=15;
            else
                obj.cost=100;
        }
        else {
            if(obj.st.objeto=='D')
                obj.cost=1;
            else
                obj.cost=3;
        }
    }
    else if(casilla=='T')
        obj.cost=2;
    else
        obj.cost=1;

    if(casilla=='X')
        obj.cost=-5;

    obj.cost+=1;
    obj.bat += obj.cost;

    obj.total_cost += obj.cost;
}


// ----------------------------------------- IMPLEMENTACIÓN ALGORITMO A* [NIVEL 3] ------------------------------------------

/**
 * Mi primera aproximación al ejercicio lo resolví con el algoritmo de Krusgal. Pero debido a la complejidad temporal
 * se me ocurrió intentar utilizar A* aplicando una heurística adecuada (Reduciendo la complejidad temporal a O(b^d)).
 * PD: Si el factor de optimización es nulo, es simplemente costeUniforme y no A*.
 * En este caso lo que hago es ir buscando los objetivos y aplicando una función coste sobre los movimientos.
 * Aun así me produce cierta incertidumbre sobre si para todos los casos la solución es la más óptima.
 * Diff: Krugal => Garante óptimibilida \\ A* => Encuentra una solución buena en un tiempo adecuado.
 * @param origen de dónde partimos
 * @param destino a dónde vamos
 * @param plan camino a seguir
 * @param FactorOptimizacion Factor para la heurística A*, indica cuánto vale la distancia hacia el objetivo
 * @param totalCost coste total de seguir el camino.
 * @return si ha encontrado o no el(los) objetivo(s)
 */
bool ComportamientoJugador::pathFinding_NObjetivos(const estado &origen, const vector<estado> &destino, list<Action> &plan,double FactorOptimizacion, double& totalCost) {
    //Borro la lista
    cout << "Calculando plan\n";
    plan.clear();
    set<estado,ComparaEstados> Cerrados; // Lista de Cerrados
    multiset<nodoCoste, ComparaCostes> Abiertos; // Lista de Abiertos

    nodoCoste current(destino.size());
    current.st = origen;
    current.st.dests.resize(destino.size());
    current.secuencia.empty();

    vector<estado> goal = destino;
    Abiertos.insert(current);
    bool found = false, change = true;
    int index = 0, max_found = 0, df, dc;
    double distance = 0, min=0 ;
    nodoCoste temp2(destino.size());
    int fil=current.st.fila, col=current.st.columna;
    unsigned char casilla = mapaResultado[fil][col];

    while (!Abiertos.empty() and !found){
        auto it = Abiertos.begin();
        while(Cerrados.find(current.st)!=Cerrados.end()){
            Abiertos.erase(it);
            it = Abiertos.begin();
            current = *it;
            if(Abiertos.empty())
                break;
        }

        Cerrados.insert(current.st);

        //Miramos que casilla estamos.
        fil = current.st.fila;
        col = current.st.columna;
        casilla = mapaResultado[fil][col];
        current.casilla = casilla;

        for (int i = 0; i < destino.size(); i++) {
            if (fil == goal[i].fila and col == goal[i].columna) {
                current.st.dests[i] = true;
            }
        }
        if (current.st.dests[index_cercano] == true)
            change = true;

        if (change) { //Recalcular el objetivo más cerca.
            change = false;
            distance = 0;
            min = numeric_limits<double>::max();
            for (int i = 0; i < goal.size(); i++) {
                if (current.st.dests[i] == false) {
                    df = abs(current.st.fila - goal[i].fila );
                    dc = abs(current.st.columna - goal[i].columna );
                    //if(df==0) df = 1;
                    //if(dc==0) dc = 0;
                    distance = df + dc;
                    if (distance < min) {
                        min = distance;
                        index_cercano = i;
                    }
                }
            }
        } else {
            distance = abs(current.st.fila - goal[index_cercano].fila);
            distance += abs(current.st.columna - goal[index_cercano].columna);
            min = distance;
        }

        // Generar descendiente de girar a la derecha
        nodoCoste hijoTurnR = current;
        hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion + 1) % 4;
        if (Cerrados.find(hijoTurnR.st) == Cerrados.end()) {
            hijoTurnR.secuencia.push_back(actTURN_R);
            Cost2(casilla, hijoTurnR, goal, 0);
            Abiertos.insert(hijoTurnR);
        }

        // Generar descendiente de girar a la izquierda
        nodoCoste hijoTurnL = current;
        hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion + 3) % 4;
        if (Cerrados.find(hijoTurnL.st) == Cerrados.end()) {
            hijoTurnL.secuencia.push_back(actTURN_L);
            Cost2(casilla, hijoTurnL, goal, 0);
            Abiertos.insert(hijoTurnL);
        }

        // Generar descendiente de avanzar
        nodoCoste hijoForward = current;
        if (!HayObstaculoDelante(hijoForward.st)) {
            if (Cerrados.find(hijoForward.st) == Cerrados.end()) {
                fil = hijoForward.st.fila;
                col = hijoForward.st.columna;
                casilla = mapaResultado[fil][col];
                EsObjeto(hijoForward.st);
                hijoForward.secuencia.push_back(actFORWARD);

                Cost2(casilla, hijoForward, goal, FactorOptimizacion);
                Abiertos.insert(hijoForward);
            }
        }

        // Tomo el siguiente valor de la Abiertos
        if (!Abiertos.empty()) {
            int f = 0;
            for (int i = 0; i < current.st.dests.size(); i++) {
                if (current.st.dests[i])
                    f++;
            }
            if (f > max_found) {
                max_found = f;
                temp2 = current;
            }
            if (max_found == goal.size())
                break;
            it = Abiertos.begin();
            current = *it;
        }
    }

    cout << "Terminada la busqueda\n";
    if(Abiertos.empty())
        cout << "\n\t[EMPTY]\n\n";

    for(int i=0;i<temp2.st.dests.size();i++) { //for debuggin purposes me quedo siempre con la última mejor solución.
        if (temp2.st.dests[i]==true) {
            cout << "Cargando el plan\n";
            cout << "[OVERALL COST]: " << temp2.total_cost << ((FactorOptimizacion>0)?" DISTORTED VALUE":" BATTERY VALUE") << endl;
            totalCost = temp2.total_cost;
            total_cost = totalCost;
            plan = temp2.secuencia;
            cout << "Longitud del plan: " << plan.size() << endl;

            return true;
        }
    }
    if(!found){
        cout << "No encontrado plan\n";
    }


    return false;
}
// ----------------------------------------- IMPLEMENTACIÓN ALGORITMO KRUSGAL [NIVEL 3] ------------------------------------------
/**
 * Pensé que para poder asegurar que el resultado era 100% el camino más óptimo era necesario entonces simular el grafo
 * de manera que podría calcular el arbol generador mínimo. Sin embargo, aunque el algoritmo funciona, la complexidad
 * temporal es bastante elevada, sino me equivoco de O(n!*b^d). Entonces fue cuando desenvolví el otro algoritmo A* con
 * una heurística adecuada.
 * @param origen de dónde partimos
 * @param destino a dónde vamos
 * @param plan camino a seguir
 * @param FactorOptimizacion Factor para la heurística A*, indica cuánto vale la distancia hacia el objetivo
 * @param totalCost coste total de seguir el camino.
 * @return si ha encontrado o no el(los) objetivo(s)
 */
bool ComportamientoJugador::pathFinding_Krusgal(const estado &origen, const vector<estado> &destino, list<Action> &plan,double FactorOptimizacion, double& totalCost) {
    double cost[destino.size()] = {0};
    bool found[destino.size()] = {0};
    vector<list<Action>>planes;
    planes.resize(destino.size());
    vector<estado> goal = destino;
    vector<int> indexes;
    goal[0].orientacion = origen.orientacion;
    int index = 0;
    for(int x=0;x<destino.size();x++){
        min_distance = numeric_limits<double>::max(); //RESET MIN
        for(int i=0;i<destino.size();i++){
            auto it = find(indexes.begin(), indexes.end(),i);
            if(it==indexes.end()){ //NO MATCH
                if(x==0)
                    found[i] = pathFinding_CosteUniforme(goal[0],goal[i+1],plan,FactorOptimizacion,cost[i]);
                else if(!indexes.empty()){
                    found[i] = pathFinding_CosteUniforme(goal[indexes[x-1]+1],goal[i+1], plan,FactorOptimizacion,cost[i]);
                }
                if(cost[i] < min_distance) {
                    min_distance = cost[i]; //UPDATE MIN
                    index_cercano = i;
                }
                if(found[i]) {
                    planes[i].clear();
                    planes[i] = plan; //STORE PLAN IF FOUND
                }
            }
        }
        indexes.push_back(index_cercano);
        if(x!=0)
            planes[indexes[0]].insert(planes[indexes[0]].end(),planes[index_cercano].begin(),planes[index_cercano].end()); //Concatenate plans
        totalCost += cost[index_cercano];
    }
    plan.clear();
    plan = planes[indexes[0]];

    cout << "[FINISHED ALIVE WITH COST]: " << totalCost << ((FactorOptimizacion>0)?" Distorted Value":" BatteryValue") << endl;
    return found[indexes[0]];
}


void ComportamientoJugador::rellenaMapa(estado st,vector<unsigned char> terreno){
    int fil = st.fila;
    int col = st.columna;
    int cont = 0;
    int x = 1;
    switch(st.orientacion){
        case 0:
            for(int i=0;i<4;i++){
                for(int j=0;j<i+x;j++){
                    if(fil-i>=0 and fil-i<mapaResultado.size() and col-i+j>=0 and col-i+j<mapaResultado[fil-i].size())
                        mapaResultado[fil-i][col-i+j] = terreno[cont];
                    cont++;
                    if(i==0)
                        break;
                }
                x++;
            }
            break;
        case 1:
            for(int i=0;i<4;i++){
                for(int j=0;j<i+x;j++){
                    if(fil-i+j>=0 and fil-i+j<mapaResultado.size() and col-i>=0 and col-i<mapaResultado[fil-i+j].size())
                        mapaResultado[fil-i+j][col+i] = terreno[cont];
                    cont++;
                    if(i==0)
                        break;
                }
                x++;
            }
            break;
        case 2:
            for(int i=0;i<4;i++){
                for(int j=0;j<i+x;j++){
                    if(fil+i>=0 and fil+i<mapaResultado.size() and col-i+j>=0 and col-i+j<mapaResultado[fil+i].size())
                        mapaResultado[fil+i][col+i-j] = terreno[cont];
                    cont++;
                    if(i==0)
                        break;
                }
                x++;
            }
            break;
        case 3:
            for(int i=0;i<4;i++){
                for(int j=0;j<i+x;j++){
                    if(fil+i-j>=0 and fil+i-j<mapaResultado.size() and col-i>=0 and col-i<mapaResultado[fil+i-j].size())
                        mapaResultado[fil+i-j][col-i] = terreno[cont];
                    cont++;
                    if(i==0)
                        break;
                }
                x++;
            }
            break;
    }
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------- RESOLUCIÓN DEL NIVEL 0 ----------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

//---------------------- Implementación de la busqueda en Profundidad ---------------------------

bool ComportamientoJugador::pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan) {
    //Borro la lista
    cout << "Calculando plan\n";
    plan.clear();
    set<estado,ComparaEstados> Cerrados; // Lista de Cerrados
    stack<nodo> Abiertos;								 // Lista de Abiertos

    nodo current;
    current.st = origen;
    current.secuencia.empty();

    Abiertos.push(current);

    while (!Abiertos.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

        Abiertos.pop();
        Cerrados.insert(current.st);

        // Generar descendiente de girar a la derecha
        nodo hijoTurnR = current;
        hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
        if (Cerrados.find(hijoTurnR.st) == Cerrados.end()){
            hijoTurnR.secuencia.push_back(actTURN_R);
            Abiertos.push(hijoTurnR);

        }

        // Generar descendiente de girar a la izquierda
        nodo hijoTurnL = current;
        hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
        if (Cerrados.find(hijoTurnL.st) == Cerrados.end()){
            hijoTurnL.secuencia.push_back(actTURN_L);
            Abiertos.push(hijoTurnL);
        }

        // Generar descendiente de avanzar
        nodo hijoForward = current;
        if (!HayObstaculoDelante(hijoForward.st)){
            if (Cerrados.find(hijoForward.st) == Cerrados.end()){
                hijoForward.secuencia.push_back(actFORWARD);
                Abiertos.push(hijoForward);
            }
        }

        // Tomo el siguiente valor de la Abiertos
        if (!Abiertos.empty()){
            current = Abiertos.top();
        }
    }

    cout << "Terminada la busqueda\n";

    if (current.st.fila == destino.fila and current.st.columna == destino.columna){
        cout << "Cargando el plan\n";
        plan = current.secuencia;
        cout << "Longitud del plan: " << plan.size() << endl;

        return true;
    }
    else {
        cout << "No encontrado plan\n";
    }


    return false;
}


// Sacar por la consola la secuencia del plan obtenido
void ComportamientoJugador::PintaPlan(list<Action> plan) {
    auto it = plan.begin();
    while (it!=plan.end()){
        if (*it == actFORWARD){
            cout << "A ";
        }
        else if (*it == actTURN_R){
            cout << "D ";
        }
        else if (*it == actTURN_L){
            cout << "I ";
        }
        else {
            cout << "- ";
        }
        it++;
    }
    cout << endl;
}


// Funcion auxiliar para poner a 0 todas las casillas de una matriz
void AnularMatriz(vector<vector<unsigned char> > &m){
    for (int i=0; i<m[0].size(); i++){
        for (int j=0; j<m.size(); j++){
            m[i][j]=0;
        }
    }
}


// Pinta sobre el mapa del juego el plan obtenido
void ComportamientoJugador::VisualizaPlan(const estado &st, const list<Action> &plan){
    AnularMatriz(mapaConPlan);
    estado cst = st;

    auto it = plan.begin();
    while (it!=plan.end()){
        if (*it == actFORWARD){
            switch (cst.orientacion) {
                case 0: cst.fila--; break;
                case 1: cst.columna++; break;
                case 2: cst.fila++; break;
                case 3: cst.columna--; break;
            }
            mapaConPlan[cst.fila][cst.columna]=1;
        }
        else if (*it == actTURN_R){
            cst.orientacion = (cst.orientacion+1)%4;
        }
        else if(*it == actTURN_L){
            cst.orientacion = (cst.orientacion+3)%4;
        }
        it++;
    }
}



int ComportamientoJugador::interact(Action accion, int valor){
    return false;
}
