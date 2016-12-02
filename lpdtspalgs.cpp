/*******************************************************************************
 * MC658 - Projeto e Análise de Algoritmos III - 2s2016
 * Prof.: Flavio Keidi Miyazawa
 * PED: Mauro Henrique Mulati
 ******************************************************************************/

/* IMPLEMENTE AS FUNCOES INDICADAS
 * DIGITE SEU RA: 123153
 * SUBMETA SOMENTE ESTE ARQUIVO */

#include <iostream>
#include <float.h>
#include <lemon/list_graph.h>
#include "mygraphlib.h"
#include "lpdtspalgs.h"

bool naive(const LpdTspInstance &l, LpdTspSolution  &s, int tl);

DNode find_source(const LpdTspInstance &l, vector<DNode> pickups, int item)
{
   DNode node;
   for(int i = 0; i < pickups.size(); i++){
      if(l.s[ pickups[i] ] == item){
         node = pickups[i];
         break;
      }
   }
   return node;
}

DNode find_destination(const LpdTspInstance &l, vector<DNode> deliveries, int item)
{
   DNode node;
   for(int i = 0; i < deliveries.size(); i++){
      if(l.t[ deliveries[i] ] == item){
         node = deliveries[i];
         break;
      }
   }
   return node;
}

bool remove_node(vector<DNode> &list, DNode node)
{
   for(auto i = list.begin(); i != list.end(); ++i){
      if(*i == node){
         list.erase(i);
         return true;
      }
   }
   return false;
}

bool contains_node(vector<DNode> &list, DNode node)
{
   for(auto i = list.begin(); i != list.end(); ++i){
      if(*i == node){
         return true;
      }
   }
   return false;
}

void populate_selection_without_last_node(const LpdTspInstance &l, vector<DNode> &selection, DNodeIntMap &h, vector<DNode> pickups, vector<DNode> deliveries)
{
   if(selection.size() == 0){
      for(int i = 0; i < pickups.size(); i++){
         if (!h[ pickups[i] ]){
            selection.push_back(pickups[i]);
            break;
         }
      }
   }
   if(selection.size() == 0){
      for(int i = 0; i < deliveries.size(); i++){
         if (h[ find_source(l, pickups, l.t[ deliveries[i] ])] && !h[deliveries[i]]){
            selection.push_back(deliveries[i]);
         }
      }
   }
}

void populate_selection(const LpdTspInstance &l, vector<DNode> &selection, DNodeIntMap &h, vector<DNode> pickups, vector<DNode> deliveries, DNode node)
{
   for(OutArcIt o(l.g, node); o != INVALID; ++o){
      DNode v = l.g.target(o);
      if (l.s[v] != 0 && !h[v] && !contains_node(selection, v)){
         selection.push_back(v);
      }
   }
   if(l.s[node] != 0){ // se é um nó Pickup
      int item = l.s[node];
      selection.push_back(find_destination(l, deliveries, item));
   }
   
   if(selection.size() == 0){
      for(int i = 0; i < pickups.size(); i++){
         if (!h[ pickups[i] ]){
            selection.push_back(pickups[i]);
            break;
         }
      }
   }
   if(selection.size() == 0){
      for(int i = 0; i < deliveries.size(); i++){
         if (h[find_source(l, pickups, l.t[ deliveries[i] ])] && !h[deliveries[i]]){
            selection.push_back(deliveries[i]);
         }
      }
   }
}


//------------------------------------------------------------------------------
bool constrHeur(const LpdTspInstance &l, LpdTspSolution  &s, int tl)
/* Implemente esta função, entretanto, não altere sua assinatura */
{

   s.tour.clear();
   s.cost = 0.0;

   double max_c1i = 0.0; // distância máxima entre o depósito e os nós Pickup adjacentes
   DNode v_i; // nó adjacente i (cuja distância c1i será máxima)

   for(OutArcIt o(l.g, l.depot); o != INVALID; ++o){
      DNode v = l.g.target(o);
      if (l.s[v] != 0 && l.weight[o] > max_c1i){
         v_i = v;
         max_c1i = l.weight[o];
      }
   }

   DNodeIntMap h(l.g); // mapa para denotar os nós que já estão no trajeto da solução

   vector<DNode> pickups = vector<DNode>(); // conjunto P de nós Pickup
   vector<DNode> deliveries = vector<DNode>(); // conjunto D de nós Delivery
   for(DNodeIt v(l.g); v != INVALID; ++v){
      h[v] = 0;
      if(l.s[v] != 0) {
         pickups.push_back(v);
      }
      if(l.t[v] != 0) {
         deliveries.push_back(v);
      }
   }
   cout << "P = {";
   for(int i = 0; i < pickups.size(); i++)
      cout << " " << l.vname[pickups[i]];
   cout << "}" << endl;
   cout << "D = {";
   for(int i = 0; i < deliveries.size(); i++)
      cout << " " << l.vname[deliveries[i]];
   cout << "}" << endl;

   h[l.depot] = 1;
   s.tour.push_back(l.depot);
   h[v_i] = 1;
   s.tour.push_back(v_i);

   cout << "s.cost += " << max_c1i << endl;
   s.cost += max_c1i;

   s.tour.push_back(l.depot); // [*] irá ser removido depois
   for(OutArcIt o(l.g, v_i); o != INVALID; ++o){
      if(l.g.target(o) == l.depot){
         cout << "s.cost += " << l.weight[o] << endl;
         s.cost += l.weight[o];
         break;
      }
   }

   vector<DNode> selection = vector<DNode>();  // conjunto S para seleção de nós

   // S = P \ { i } U { d(i) }
   // o vértice i é Pickup, d(i) é o vértice Delivery para o item coletado em i
   populate_selection(l, selection, h, pickups, deliveries, v_i);

   while(selection.size() > 0){
      cout << "BEGIN WHILE:  " << selection.size() << endl;
      cout << "S = {";
      for(int i = 0; i < selection.size(); i++)
         cout << " " << l.vname[selection[i]];
      cout << "}" << endl;
      cout << "H = {";
      for(int i = 0; i < s.tour.size(); i++)
         cout << " " << l.vname[s.tour[i]];
      cout << "}" << endl;

      double maximal_s = 0.0;
      DNode k;

      // para cara vertice i em S, 
      // vamos procurar as distâncias minimais c_ij entre i e cada vértice j já inseridos na solução
      // então iremos pegar a distância maximal c_ij... j será o vértice a ser inserido na solução
      for(int i = 0; i < selection.size(); i++){
         double minimal_h = DBL_MAX;
         DNode j;

         for(OutArcIt o(l.g, selection[i]); o != INVALID; ++o){
            if(h[l.g.target(o)] == 1 && l.weight[o] < minimal_h){
               minimal_h = l.weight[o];
               j = l.g.target(o);
            }
         }

         if(minimal_h > maximal_s){ // && !h[ selection[i] ]){
            maximal_s = minimal_h;
            k = selection[i];
         }
      }


      // vamos inserir k na solução causando o menor impacto no custo do trajeto
      std::vector<DNode>::iterator s_start_it = s.tour.begin(); // se é um nó Pickup, podemos inseri-lo em qualquer posição
      
      cout << "K =  " << l.vname[k] << endl;

      if(l.t[k] != 0){ // se é um nó Delivery, precisamos inseri-lo somente depois do seu nó Pickup
         int item = l.t[k];
         DNode pk = find_source(l, pickups, item);

         for(s_start_it = s.tour.begin(); *s_start_it != pk; s_start_it++){}
      }

      cout << "s_start_it = " << std::distance(s.tour.begin(), s_start_it) << endl;

      std::vector<DNode>::iterator k_position = s_start_it;
      double min_k_cost_insertion = DBL_MAX;

      for(std::vector<DNode>::iterator i = s_start_it; i != s.tour.end()-1; i++){
         double c_ij = 0.0, c_ik  = 0.0, c_kj = 0.0;

         for(OutArcIt o(l.g, *i); o != INVALID; ++o){
            if(l.g.target(o) == k){
               cout << "c_ik" << endl;
               c_ik = l.weight[o];

               for(InArcIt j(l.g, *(i+1)); j != INVALID; ++j){
                  if(l.g.source(j) == k){
                     cout << "c_kj" << endl;
                     c_kj = l.weight[j];
                  }
               }
            }
            if(l.g.target(o) == *(i+1)){
               cout << "c_ij" << endl;
               c_ij = l.weight[o];
            }
         }

         if (c_ik != 0.0 && c_kj != 0.0 && c_ij != 0.0
            & c_ik + c_kj - c_ij < min_k_cost_insertion){

            cout << "c_ik = " << c_ik << " / " << "c_kj = " << c_kj << " / " << "c_ij = " << c_ij << endl;
            min_k_cost_insertion = c_ik + c_kj - c_ij;
            cout << "min_k_cost_insertion = " << min_k_cost_insertion << endl;
            k_position = i + 1;
         }
      }

      if (k_position == s.tour.begin())
         k_position++;

      if (min_k_cost_insertion == DBL_MAX){
         remove_node(selection, k);
         populate_selection_without_last_node(l, selection, h, pickups, deliveries);
         continue;
      }

      cout << "K POSITION = " << std::distance(s.tour.begin(), k_position) << " / INSERTION COST " << min_k_cost_insertion << endl;

      s.tour.emplace(k_position, k);
      h[k] = 1;
      cout << "s.cost += " << min_k_cost_insertion << endl;
      s.cost += min_k_cost_insertion;

      remove_node(selection, k);
      populate_selection(l, selection, h, pickups, deliveries, k);

      cout << "END WHILE:  " << selection.size() << endl;

   }

   s.tour.erase(s.tour.end() - 1);

   cout << "H = {";
   for(int i = 0; i < s.tour.size(); i++)
      cout << " " << l.vname[s.tour[i]];
   cout << "}" << endl;

   
   return false;
}
//------------------------------------------------------------------------------
bool metaHeur(const LpdTspInstance &l, LpdTspSolution  &s, int tl)
/* Implemente esta função, entretanto, não altere sua assinatura */
{
   return naive(l, s, tl);
}
//------------------------------------------------------------------------------
bool exact(const LpdTspInstance &l, LpdTspSolution  &s, int tl)
/* Implemente esta função, entretanto, não altere sua assinatura */
{
   return naive(l, s, tl);
}
//------------------------------------------------------------------------------
bool naive(const LpdTspInstance &instance, LpdTspSolution  &sol, int tl)
/*
 * Algoritmo ingênuo para o LPD-TSP. Ideia:
 * constrNaiveHeur(l, s)
 *    s.tour.push_back(l.depot)
 *    while(s.tour.size() < 2*l.k+1)
 *       v = argmin_{v' in V} {d_{(v,v')} | (v' é adj a v) e ((v' é s) ou (v' é t de i cujo s é u em l.tour))}
 *       l.tour.push_back(v)
 */
{
   DNode v,
         vl;

   double vval,
          vlval;

   int i;

   sol.tour.clear();
   sol.cost = 0.0;

   v = instance.depot;
   sol.tour.push_back(v);

   while((int)sol.tour.size() < 2 * instance.k + 1 && v != INVALID){
      v    = INVALID;
      vval = DBL_MAX;

      for(OutArcIt o(instance.g, sol.tour.back()); o != INVALID; ++o){
         vl    = instance.g.target(o);
         vlval = DBL_MAX;

         i = 0;
         while(i < (int)sol.tour.size() && vl != sol.tour[i]) i++;
         if(i < (int)sol.tour.size()) continue;

         if(instance.s[vl] > 0){
            vlval = instance.weight[o];
         }
         else if(instance.t[vl] > 0){
            i = 0;
            while(i < (int)sol.tour.size() && instance.t[vl] != instance.s[sol.tour[i]]){
               i++;
            }
            if(i < (int)sol.tour.size()){
               vlval = instance.weight[o];
            }
         }

         if(vlval < vval){
            v    = vl;
            vval = vlval;
         }
      }

      if(v != INVALID){
         sol.tour.push_back(v);
         sol.cost += vval;
      }
   }

   if(v == INVALID){
      sol.cost = DBL_MAX;
   }
   else{
      OutArcIt o(instance.g, sol.tour.back());
      for(; o != INVALID; ++o){
         if(instance.g.target(o) == sol.tour.front()) break;
      }
      if(o != INVALID){
         sol.cost += instance.weight[o];
      }
   }
	
	return false;
}
//------------------------------------------------------------------------------

