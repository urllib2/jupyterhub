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
         sym.stop(0);
          play=1;
         
         // Masquer un élément 
         sym.$("VS").hide();
         // Masquer un élément 
         sym.$("Gr_yeux_").hide();
         
         // Masquer un élément 
         sym.$("Gr_coeur").hide();
         
         // Masquer un élément 
         sym.$("Gr_cerveau_").hide();
         
         // Masquer un élément 
         sym.$("Gr_bouche").hide();
         
         // Masquer un élément 
         sym.$("Gr_msucles").hide();
         
         // Masquer un élément 
         sym.$("Gr_oreilles").hide();
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5500, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${muscle}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("Gr_msucles").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${muscle}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("Gr_msucles").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Ellipse2}", "mouseover", function(sym, e) {
         sym.$("Gr_coeur").show();
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Ellipse2}", "mouseleave", function(sym, e) {
         sym.$("Gr_coeur").hide();
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${mouth}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("Gr_bouche").show();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${mouth}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("Gr_bouche").hide();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${ear}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("Gr_oreilles").show();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${ear}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("Gr_oreilles").hide();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${brain}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("Gr_cerveau_").show();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${brain}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("Gr_cerveau_").hide();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${eyes}", "mouseover", function(sym, e) {
         sym.$("Gr_yeux_").show();
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${eyes}", "mouseleave", function(sym, e) {
         sym.$("Gr_yeux_").hide();
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("VS").fadeIn(1000);
         sym.$("TextCopy").fadeOut(500);
         sym.$("valider").fadeOut(500);
         sym.$("robotvshumain").fadeOut(500);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

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
   
   //Edge symbol: 'TITRE_1'
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

   })("TITRE_1");
   //Edge symbol end:'TITRE_1'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");