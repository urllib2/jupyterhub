/***********************
* Adobe Edge Animate Composition Actions
*
* Edit this file with caution, being careful to preserve 
* function signatures and comments starting with 'Edge' to maintain the 
* ability to interact with these actions from within Adobe Edge Animate
*
***********************/
(function($, Edge, compId){
var Composition = Edge.Composition, Symbol = Edge.Symbol; // aliases for commonly used Edge classes

   //Edge symbol: 'stage'
   (function(symbolName) {
      
      
      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
         sym.$("Group").hide();
         sym.$("Group2").hide();
         sym.$("Group3").hide();
         sym.$("AvancerCopy").hide();sym.$("Avancer").hide();
         sym.$("Group").delay(500).fadeIn(700);
         sym.$("AvancerCopy").delay(900).fadeIn(700);
         
         sym.$("Group2").delay(1300).fadeIn(700);
         sym.$("Avancer").delay(1900).fadeIn(700);
         sym.$("Group3").delay(2200).fadeIn(700);

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

   //=========================================================
   
   //Edge symbol: 'TITRE_1_1'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         // insérer le code ici
      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5250, function(sym, e) {
         // insérer le code ic
         sym.stop();

      });
      //Edge binding end

   })("TITRE_1_1");
   //Edge symbol end:'TITRE_1_1'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");