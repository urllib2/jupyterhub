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
         // Masquer un élément 
         sym.$("wings").hide();
         // Masquer un élément 
         sym.$("wheels").hide();
         // Masquer un élément 
         sym.$("frame").hide();
         // Masquer un élément 
         sym.$("motor").hide();
         
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${dr_wing3Copy}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         // Afficher un élément 
         sym.$("dr_wing3").fadeTo(300,1);
         
         sym.$("wings").show();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${dr_wing3Copy}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         // Masquer un élément 
         sym.$("dr_wing3").fadeTo(300,0);
         
         sym.$("wings").hide();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${dr_frame3Copy}", "mouseover", function(sym, e) {
         sym.$("dr_frame3").fadeTo(300,1);
         
         // Afficher un élément 
         sym.$("frame").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${dr_frame3Copy}", "mouseleave", function(sym, e) {
         sym.$("dr_frame3").fadeTo(300,0);
         
         // Masquer un élément 
         sym.$("frame").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${dr_wheels2Copy}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         // Afficher un élément 
         sym.$("dr_wheels2").fadeTo(300,1);
         
         // Afficher un élément 
         sym.$("wheels").show();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${dr_wheels2Copy}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         // Masquer un élément 
         sym.$("dr_wheels2").fadeTo(300,0);
         
         // Masquer un élément 
         sym.$("wheels").hide();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${dr_motor2Copy}", "mouseover", function(sym, e) {
         
         // Masquer un élément 
         sym.$("dr_motor2").fadeTo(300,1);
         
         
         sym.$("motor").show();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${dr_motor2Copy}", "mouseleave", function(sym, e) {
         
         // Afficher un élément 
         sym.$("dr_motor2").fadeTo(300,0);
         
         sym.$("motor").hide();
         
         

      });
      //Edge binding end

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'TITRE'
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

   })("TITRE");
   //Edge symbol end:'TITRE'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");