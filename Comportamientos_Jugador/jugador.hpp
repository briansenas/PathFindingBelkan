#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>
#include <set>

struct estado {
  int fila;
  int columna;
  int orientacion;
  unsigned char objeto;
  vector<bool> dests;

  estado(){
      dests.clear();
      fila = 0;
      columna = 0;
      orientacion = 0;
      objeto = '0';
  }

  estado(int n){
      dests.resize(n);
  }

  estado(const estado& b){
     fila = b.fila;
     columna = b.columna;
     orientacion = b.orientacion;
     objeto = b.objeto;
     dests = b.dests;
  }

};
struct nodo{
    estado st;
    list<Action> secuencia;

    bool operator==(nodo& b){
        return (st.fila == b.st.fila and st.columna == b.st.columna and st.orientacion==b.st.orientacion);
    }
};

struct nodoCoste : nodo{
    unsigned char casilla;
    double cost;
    double bat;
    double total_cost;

    nodoCoste(){
        st.dests.clear();
        cost = 0;
        bat = 0;
        total_cost = 0;
    }

    nodoCoste(int n){
        cost = 0;
        bat = 0;
        total_cost = 0;
        st.dests.resize(n);
    }

};

struct soloPosicion{
    bool operator()(const estado &a, const estado &n) const{
       if((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna))
            return true;
        else
            return false;
    }
};

class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      // Inicializar Variables de Estado
    }
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      // Inicializar Variables de Estado
    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);
    void VisualizaPlan(const estado &st, const list<Action> &plan);
    ComportamientoJugador * clone(){return new ComportamientoJugador(*this);}

  private:
    // Declarar Variables de Estado
    estado actual, goal;
    vector<estado> objetivos;
    list<Action> plan;
    bool hay_plan, busca_objeto, busca_bateria, Parameters, THOUGHT, charging, aldeano;
    int index_cercano, index_bateria, MINBATTERY,MAXCHARGE, ITER, THRESHOLD;
    double min_distance,total_cost, FP, diff,OLD, st_bat, exp_rate;
    Action last_action;
    set<estado,soloPosicion> baterias;

    // Métodos privados de la clase
    bool EsObjeto(estado& obj);
    bool EsObstaculo(unsigned char casilla);
    bool alrededorCostoso(estado temp);
    bool NearByCharger();
    double LookAhead(estado st);
    double DistBat(estado temp);
    bool pathFinding(Sensores sensores, const estado &origen, const vector<estado> &destino, list<Action> &plan);
    bool pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan);
    void Cost(unsigned char casilla, nodoCoste& obj, const estado destino, double FactorOptimizacion);
    bool pathFinding_CosteUniforme(const estado &origen, estado &destino, list<Action> &plan,double FactorOptimizacion, double& total_cost);
    bool pathFinding_NObjetivos(const estado &origen, const vector<estado> &destino, list<Action> &plan,double FactorOptimizacion, double& total_cost); //BETTER
    bool pathFinding_Krusgal(const estado &origen, const vector<estado> &destino, list<Action> &plan,double FactorOptimizacion, double& total_cost); //+++TIME
    void rellenaMapa(estado st, vector<unsigned char> terreno);

    void PintaPlan(list<Action> plan);
    bool HayObstaculoDelante(estado &st);

};

#endif
