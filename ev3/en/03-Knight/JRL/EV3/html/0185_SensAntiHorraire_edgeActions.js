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
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         // Masquer un élément 
         
         // Masquer un élément
         sym.$("vid").hide();
         sym.$("question").hide();
         sym.$("play1").hide();
         sym.$("pause").hide();
         sym.$('reponse').hide();
         sym.$('A2_puissnagatif').hide()
         play=1;
         hint=1;
         demo=1;
         sym.$("Text4").hide();
         
         // Masquer un élément 
         sym.$("Solution_Ex1").hide();
         
         
         
         // Masquer un élément 
         
         
         // Masquer un élément 
         sym.$("reflechir").hide();
         
         // Masquer un élément 
         sym.$("Rectangle2").hide();
         sym.$("solu").hide();
         
         // Masquer un élément 
         sym.$("solution").hide();
         
         sym.$("demonstration").hide();
         // Masquer un élément 
         
         
         
         
         sym.play();
         
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

            



      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 883, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2003, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${demo}", "click", function(sym, e) {
         
         sym.$("question").fadeIn(800);
         sym.$("positif").fadeOut(500);
         sym.$("Intro").fadeOut(500);
         sym.$("Explication").hide();
         sym.$("demo").fadeOut(200);
         
         setTimeout(function(){ sym.$("demo").fadeIn(100);sym.$("Text4").fadeIn(100);sym.$("Text3").hide();},10000);
         
          clearTimeout();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text4}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$('A2_puissnagatif').fadeIn(600);
         
         sym.$('demo').hide()

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${question}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         

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

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5000, function(sym, e) {
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