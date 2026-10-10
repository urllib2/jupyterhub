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
         sym.stop();
         sym.$('Text').hide();
         sym.$('Text3').hide();
         sym.$('Text2').hide();
         
         nb=0;
         
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 6000, function(sym, e) {
         // insérer le code ici
         sym.stop()

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4750, function(sym, e) {
         // insérer le code ici
         sym.stop()

      });
      //Edge binding end

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 750, function(sym, e) {
         // insert code here
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Symbol_1}", "click", function(sym, e) {
         // insert code for mouse click here
         sym.getSymbol("Symbol_1").play(0);
         
         
         
         
         sym.$("Description2").hide();
         nb=nb+1;
         
         if(nb==1)
         {
         sym.getSymbol("Box").play(0);
         sym.$("pointer").hide();
         sym.$("Description").html("Savoir si le robot rencontre un obstacle");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("Le karting");
         }
         else if(nb==2)
         {
         sym.getSymbol("Box").play(3000);
         sym.$("Description").html("Déclencher une action.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("La Formule1");
         }
         
         else if(nb==3)
         {
         sym.getSymbol("Box").play(6000);
         sym.$("Description").html("Détecter si un mécanisme dans le robot a atteint une certaine position.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("Le rallye");
         nb=0;
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Symbol_1}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         sym.$("Symbol_1").fadeIn(300)

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

   //=========================================================
   
   //Edge symbol: 'Box'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 7500, function(sym, e) {
         // insert code here
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4500, function(sym, e) {
         // insert code here
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1500, function(sym, e) {
         // insert code here
         sym.stop();

      });
      //Edge binding end

   })("Box");
   //Edge symbol end:'Box'

   //=========================================================
   
   //Edge symbol: 'Next'
   (function(symbolName) {   
   
   })("Next");
   //Edge symbol end:'Next'

   //=========================================================
   
   //Edge symbol: 'Symbol_1'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 500, function(sym, e) {
         // insert code here
         sym.stop();

      });
      //Edge binding end

   })("Symbol_1");
   //Edge symbol end:'Symbol_1'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");