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
      
      
      Symbol.bindElementAction(compId, symbolName, "${_3}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // Masquer un élément 
         sym.$("Text2").hide();
         
         // Masquer un élément 
         sym.$("bloc").hide();
         
         // Afficher un élément 
         sym.$("hommes").show();
         
         // Afficher un élément 
         sym.$("Text7").show();
         
         // Masquer un élément 
         sym.$("Text3").hide();
         
         // Masquer un élément 
         sym.$("ordre").hide();
         
         // Masquer un élément 
         
         
         // Afficher un élément 
         sym.$("b3").show();
         
         
         
         // Masquer un élément 
         
         sym.$("b4").hide();
         sym.$("Text9").hide();
         sym.$("etape4").hide();
         sym.$("_4Copy").hide();
         
         
         
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(2250);
         
         
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${_2}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // Masquer un élément 
         
         
         // Masquer un élément 
         sym.$("ordre").hide();
         sym.$("cach3").hide();
         
         // Afficher un élément 
         sym.$("b2").show();
         
         // Afficher un élément 
         sym.$("bloc").show();
         
         // Masquer un élément 
         sym.$("Text7").hide();
         
         // Afficher un élément 
         sym.$("bloc").show();
         
         // Afficher un élément 
         sym.$("Text3").show();
         
         // Masquer un élément 
         sym.$("hommes").hide();
         
         // Masquer un élément 
         sym.$("b3").hide();
         
         // Masquer un élément 
         sym.$("Text2").hide();
         
         sym.$("b4").hide();
         sym.$("Text9").hide();
         sym.$("etape4").hide();
         sym.$("_3Copy").hide();
         
         
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(1500);
         
         
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${_1}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // Afficher un élément 
         sym.$("b1").show();
         sym.$("retro-robot-vector-illustration").hide();
         
         // Afficher un élément 
         sym.$("Text2").show();
         sym.$("pointer").hide();
         sym.$("cach2").hide();
         // Afficher un élément 
         sym.$("ordre").show();
         
         // Masquer un élément 
         sym.$("b3").hide();
         
         // Masquer un élément 
         sym.$("hommes").hide();
         
         // Masquer un élément 
         sym.$("Text3").hide();
         
         // Masquer un élément 
         sym.$("bloc").hide();
         
         // Masquer un élément 
         sym.$("Text7").hide();
         
         // Masquer un élément 
         sym.$("b2").hide();
         sym.$("b4").hide();
         sym.$("Text9").hide();
         sym.$("etape4").hide();
         sym.$("_2Copy").hide();
         
         
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(250);
         
         
         }
         
         // Masquer un élément 
         sym.$("full_screen").hide();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
          
          play=1;
         sym.$("hommes").hide();
         sym.$("retro-robot-vector-illustration").show();
         sym.$("etape4").hide();
         // Masquer un élément 
         sym.$("ordre").hide();
         
         // Masquer un élément 
         sym.$("Text2").hide();
         
         // Masquer un élément 
         sym.$("Text3").hide();
         
         // Masquer un élément 
         sym.$("bloc").hide();
         
         // Masquer un élément 
         x=-1;
         // Masquer un élément 
         sym.$("Text7").hide();
         sym.$("Text9").hide();
         // Masquer un élément 
         sym.$("b1").hide();
         
         // Masquer un élément 
         sym.$("b2").hide();
         
         // Masquer un élément 
         sym.$("b3").hide();
         sym.$("b4").hide();
         
         
         
         sym.stop(0);
         
         // Masquer un élément 
         sym.$("full_screen").hide();

      });
      //Edge binding end

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1250, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${b1}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         // Afficher un élément 
         sym.$("b1").show();
         
         // Afficher un élément 
         sym.$("Text2").show();
         
         // Afficher un élément 
         sym.$("ordre").show();
         
         // Masquer un élément 
         sym.$("b3").hide();
         
         // Masquer un élément 
         sym.$("hommes").hide();
         
         // Masquer un élément 
         sym.$("Text3").hide();
         
         // Masquer un élément 
         sym.$("bloc").hide();
         
         // Masquer un élément 
         sym.$("Text7").hide();
         
         // Masquer un élément 
         sym.$("b2").hide();
         sym.$("b4").hide();
         sym.$("Text9").hide();
         sym.$("etape4").hide();
         sym.$("_2Copy").hide();
         sym.$("_3Copy").show();
         sym.$("_4Copy").show();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(250);
         
         
         }
         
         
         // Masquer un élément 
         sym.$("full_screen").hide();
         	sym.$("Oz_test")[0].pause();
         	sym.$("Oz_test")[0].currentTime = 0;
         

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