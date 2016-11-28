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

//------------------------------------------------------------------------------
bool constrHeur(const LpdTspInstance &l, LpdTspSolution  &s, int tl)
/* Implemente esta função, entretanto, não altere sua assinatura */
{
   return naive(l, s, tl);
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

