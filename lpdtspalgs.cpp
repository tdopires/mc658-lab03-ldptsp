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
#include <math.h>
#include <lemon/list_graph.h>
#include "mygraphlib.h"
#include "lpdtspalgs.h"

bool naive(const LpdTspInstance &l, LpdTspSolution &s, int tl);

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
   
   populate_selection_without_last_node(l, selection, h, pickups, deliveries);
}


//------------------------------------------------------------------------------
bool constrHeur_v0_farthest_insertion(const LpdTspInstance &l, LpdTspSolution  &s, int tl)
{
   clock_t beginExec = clock();

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

   h[l.depot] = 1;
   s.tour.push_back(l.depot);
   h[v_i] = 1;
   s.tour.push_back(v_i);

   s.cost += max_c1i;

   s.tour.push_back(l.depot); // [*] irá ser removido depois
   for(OutArcIt o(l.g, v_i); o != INVALID; ++o){
      if(l.g.target(o) == l.depot){
         s.cost += l.weight[o];
         break;
      }
   }

   vector<DNode> selection = vector<DNode>();  // conjunto S para seleção de nós

   // S = P \ { i } U { d(i) }
   // o vértice i é Pickup, d(i) é o vértice Delivery para o item coletado em i
   populate_selection(l, selection, h, pickups, deliveries, v_i);

   while(selection.size() > 0){
      clock_t now = clock();
      if (( (now - beginExec) / CLOCKS_PER_SEC) > tl) {
         s.tour.clear();
         s.cost = DBL_MAX;
         return false;
      }

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
      
      if(l.t[k] != 0){ // se é um nó Delivery, precisamos inseri-lo somente depois do seu nó Pickup
         int item = l.t[k];
         DNode pk = find_source(l, pickups, item);

         for(s_start_it = s.tour.begin(); *s_start_it != pk; s_start_it++){}
      }

      std::vector<DNode>::iterator k_position = s_start_it;
      double min_k_cost_insertion = DBL_MAX;

      for(std::vector<DNode>::iterator i = s_start_it; i != s.tour.end()-1; i++){
         double c_ij = 0.0, c_ik  = 0.0, c_kj = 0.0;

         for(OutArcIt o(l.g, *i); o != INVALID; ++o){
            if(l.g.target(o) == k){
               c_ik = l.weight[o];

               for(InArcIt j(l.g, *(i+1)); j != INVALID; ++j){
                  if(l.g.source(j) == k){
                     c_kj = l.weight[j];
                  }
               }
            }
            if(l.g.target(o) == *(i+1)){
               c_ij = l.weight[o];
            }
         }

         if (c_ik != 0.0 && c_kj != 0.0 && c_ij != 0.0
            & c_ik + c_kj - c_ij < min_k_cost_insertion){

            min_k_cost_insertion = c_ik + c_kj - c_ij;
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

      s.tour.emplace(k_position, k);
      h[k] = 1;
      s.cost += min_k_cost_insertion;

      remove_node(selection, k);
      populate_selection(l, selection, h, pickups, deliveries, k);
   }

   s.tour.erase(s.tour.end() - 1);

   return false;
}

bool v1_f_insertion(const LpdTspInstance &l, LpdTspSolution &s, DNodeIntMap &h, vector<int> &items_status, double &capacityCheck, clock_t beginExec, int tl)
{
   if (((clock() - beginExec) / CLOCKS_PER_SEC) > (tl-0.01))
      return false;

   if (s.tour.size() == 2 * l.k + 1)
      return true;

   DNode node_to_insert = INVALID;
   double node_to_insert_cost = 0.0;
   
   bool find_node_to_insert = true;
   vector<DNode> black_list = vector<DNode>();

   while (find_node_to_insert) {
      DNode min_delivery_node = INVALID;
      DNode min_pickup_node = INVALID;
      double min_delivery_cost = DBL_MAX;
      double min_pickup_cost = DBL_MAX;

      for(OutArcIt o(l.g, s.tour.back()); o != INVALID; ++o){
         DNode targetNode = l.g.target(o);

         if (h[targetNode] || contains_node(black_list, targetNode))
            continue;

         int s = l.s[ targetNode ];
         // is a Pickup node and item was not picked up and item weight will not be over capacity
         if(s != 0 && items_status[s-1] == 0 && capacityCheck + l.items[s-1].w <= l.capacity && l.weight[o] < min_pickup_cost) {
            min_pickup_cost = l.weight[o];
            min_pickup_node = targetNode;
         }

         int t = l.t[ targetNode ];
         // is a Delivery node and item was already picked up
         if(t != 0 && items_status[t-1] == 1 && l.weight[o] < min_delivery_cost) {
            min_delivery_cost = l.weight[o];
            min_delivery_node = targetNode;
         }
      }

      if (min_delivery_node != INVALID || min_pickup_node != INVALID) {
         double r = ((double) rand() / (RAND_MAX)); // r is a random probability
         if (min_pickup_node == INVALID)
            r = 0.0; // if we just have a delivery node and don't have a pickup node
         else if (min_delivery_node == INVALID)
            r = 1.0; // if we just have a pickup node and don't have a delivery node

         if (r < 0.5) {
            int itemNo = l.t[ min_delivery_node ] - 1;
            capacityCheck -= l.items[itemNo].w;
            items_status[itemNo] = 2;

            node_to_insert = min_delivery_node;
            node_to_insert_cost = min_delivery_cost;
         } else {
            int itemNo = l.s[ min_pickup_node ] - 1;
            capacityCheck += l.items[itemNo].w;
            items_status[itemNo] = 1;

            node_to_insert = min_pickup_node;
            node_to_insert_cost = min_pickup_cost;
         }

         s.tour.push_back(node_to_insert);
         h[node_to_insert] = 1;
         s.cost += node_to_insert_cost;

         bool insertion_ok = v1_f_insertion(l, s, h, items_status, capacityCheck, beginExec, tl);
         if (!insertion_ok) {
            s.tour.erase(s.tour.end() - 1);
            h[node_to_insert] = 0;
            s.cost -= node_to_insert_cost;

            if (r <= 0.5) {
               int itemNo = l.t[ min_delivery_node ] - 1;
               capacityCheck += l.items[itemNo].w;
               items_status[itemNo] = 1;
            } else {
               int itemNo = l.s[ min_pickup_node ] - 1;
               capacityCheck -= l.items[itemNo].w;
               items_status[itemNo] = 0;
            }
            
            black_list.push_back(node_to_insert);
         } else {
            return true;
         }
      } else {
         find_node_to_insert = false;
      }
   }
   
   // we didn't find a valid node to insert
   return false;
}

//------------------------------------------------------------------------------
bool constrHeur_v1_f_insertion(const LpdTspInstance &l, LpdTspSolution  &s, int tl)
{
   clock_t beginExec = clock();

   s.tour.clear();
   s.cost = 0.0;

   DNodeIntMap h(l.g);
   for(DNodeIt v(l.g); v != INVALID; ++v){
      h[v] = 0;
   }

   s.tour.push_back(l.depot);
   h[l.depot] = 1;

   vector<int> items_status(l.k, 0); // 0 para item não pego, 1 para item pego, 2 para entregue
   double capacityCheck = 0.0;

   v1_f_insertion(l, s, h, items_status, capacityCheck, beginExec, tl);

   for (OutArcIt o(l.g, s.tour.back()); o != INVALID; ++o){
      if (l.g.target(o) == l.depot) {
         s.cost += l.weight[o];
      }
   }

   return false;
}

//------------------------------------------------------------------------------
bool constrHeur(const LpdTspInstance &l, LpdTspSolution  &s, int tl)
/* Implemente esta função, entretanto, não altere sua assinatura */
{
   return constrHeur_v1_f_insertion(l, s, tl);
}

void insert_node_i_on_2_opt_tour(const LpdTspInstance &l, vector<DNode> tour, int i, vector<DNode> &neighbor_sol, double &neighbor_sol_cost)
{
   if (neighbor_sol.size() != 0) {
      DNode lastNode = neighbor_sol.back();

      for (OutArcIt o(l.g, lastNode); o != INVALID; ++o){
         if (l.g.target(o) == tour[i]) {
            neighbor_sol_cost += l.weight[o];
            neighbor_sol.push_back(tour[i]);
         }
      }
   } else {
      neighbor_sol.push_back(tour[i]);
   }
}


bool t_2_opt(const LpdTspInstance &l, vector<DNode> tour, int ti, int tj, vector<DNode> &neighbor_sol, double &neighbor_sol_cost)
{
   neighbor_sol.clear();
   neighbor_sol_cost = 0.0;

   for(int i = 0; i < ti; i++){
      insert_node_i_on_2_opt_tour(l, tour, i, neighbor_sol, neighbor_sol_cost);
   }

   for(int i = tj; i >= ti; i--){
      insert_node_i_on_2_opt_tour(l, tour, i, neighbor_sol, neighbor_sol_cost);
   }

   for(int i = tj + 1; i < tour.size(); i++){
      insert_node_i_on_2_opt_tour(l, tour, i, neighbor_sol, neighbor_sol_cost);
   }

   DNode lastNode = neighbor_sol.back();
   for (OutArcIt o(l.g, lastNode); o != INVALID; ++o){
      if (l.g.target(o) == l.depot) {
         neighbor_sol_cost += l.weight[o];
      }
   }

   if(neighbor_sol.size() != tour.size())
      return false;

   if (neighbor_sol[0] != l.depot)
      return false;

   vector<int> items_status(l.k, 0); // 0 para item não pego, 1 para item pego, 2 para entregue

   double currentItemCapacity = 0.0;

   for(int i = 0; i < neighbor_sol.size(); i++) {
      if(l.s[ neighbor_sol[i] ] != 0){ // is a Pickup node
         int itemNo = l.s[ neighbor_sol[i] ] - 1;
         if (items_status[itemNo] == 0) {
            currentItemCapacity += l.items[itemNo].w;
            items_status[itemNo] = 1;
         } else {
            return false;
         }
      } else if(l.t[ neighbor_sol[i] ] != 0){ // is a Delivery node
         int itemNo = l.t[ neighbor_sol[i] ] - 1;
         if (items_status[itemNo] == 1) {
            currentItemCapacity -= l.items[itemNo].w;
            items_status[itemNo] = 2;
         } else {
            return false;
         }
      }
      
      if(currentItemCapacity > l.capacity){
         return false;
      }
   }

   for(int i = 0; i < l.k; i++) {
      if (items_status[i] != 2) {
         return false;
      }
   }
   return true;
}

bool get_neighbor_solution(const LpdTspInstance &l, LpdTspSolution &s, vector<DNode> &soll, double &soll_cost)
{
   bool found_neighbor = false;
   double min_neighbor_sol_cost = DBL_MAX;

   for (int i = 0; i < s.tour.size(); i++) {
      for (int j = i+1; j < s.tour.size(); j++) {
         vector<DNode> neighbor_sol = vector<DNode>();
         double neighbor_sol_cost = 0.0;
         if (t_2_opt(l, s.tour, i, j, neighbor_sol, neighbor_sol_cost)) {
            
            if (neighbor_sol_cost < min_neighbor_sol_cost) {
               min_neighbor_sol_cost = neighbor_sol_cost;

               found_neighbor = true;
               soll = neighbor_sol;
               soll_cost = neighbor_sol_cost;
            }

         }
      }
   }
   return found_neighbor;
}

//------------------------------------------------------------------------------
bool metaHeur(const LpdTspInstance &l, LpdTspSolution &s, int tl)
/* Implemente esta função, entretanto, não altere sua assinatura */
{
   clock_t beginExec = clock();

   double temperature = 10.0;
   double k = 1.0;
   double alpha = 0.95;

   double temperature_stop = 0.001;
   int internal_loop_stop = 100;   

   constrHeur(l, s, tl);

   vector<DNode> sol = s.tour;
   double sol_cost = s.cost;

   vector<DNode> best_sol = sol;
   double best_sol_cost = sol_cost;

   while(temperature > temperature_stop && ((clock() - beginExec) / CLOCKS_PER_SEC) < tl ){ // loop externo

      int no_change_loops = 0;
      while(no_change_loops < internal_loop_stop && ((clock() - beginExec) / CLOCKS_PER_SEC) < tl ){
         vector<DNode> soll = vector<DNode>();
         double soll_cost = 0.0;
         if (get_neighbor_solution(l, s, soll, soll_cost)){
            double delta = soll_cost - sol_cost;
            if (delta < 0) {
               no_change_loops = 0;

               sol = soll;
               sol_cost = soll_cost;

               if (sol_cost < best_sol_cost){
                  best_sol = sol;
                  best_sol_cost = sol_cost;
               }
            } else {
               double prob = exp(- delta / k * temperature);
               double r = ((double) rand() / (RAND_MAX));
               if (r <= prob){
                  no_change_loops = 0;

                  sol = soll;
                  sol_cost = soll_cost;
               } else {
                  no_change_loops++;
               }
            }
         } else {
            no_change_loops++;
         }
      }
      temperature = temperature * alpha;
   }

   s.tour = best_sol;
   s.cost = best_sol_cost;

   return false;
}
//------------------------------------------------------------------------------
bool exact(const LpdTspInstance &l, LpdTspSolution &s, int tl)
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

         if(instance.s[vl] > 0){  // If DNode vl is start of an item
            vlval = instance.weight[o];
         }
         else if(instance.t[vl] > 0){  // If DNode vl is término of an item
            i = 0;
            while(i < (int)sol.tour.size() && instance.t[ vl ] != instance.s[ sol.tour[i] ]){  // Look for the start DNode of the item which terminates in DNode vl
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

