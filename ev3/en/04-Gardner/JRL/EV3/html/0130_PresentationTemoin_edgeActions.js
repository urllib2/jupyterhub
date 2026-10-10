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
         sym.$("Text3").show();
         sym.$("Text1").hide();
         sym.$("pointer").show();
         
         // Afficher un élément 
         sym.$("b3").show();
         
         sym.$("rendu")[0].currentTime = 7.25;
         if (sym.$("rendu")[0].paused) {
         	sym.$("rendu")[0].play();
         }
         
         sym.$("rendu")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("rendu")[0].currentTime >=9){
           sym.$("rendu")[0].pause();}
         
         
           }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${_2}", "click", function(sym, e) {
         
         // Afficher un élément 
         sym.$("b1").show();
         sym.$("b2").show();
         sym.$("b3").hide();
         sym.$("Rectangle3").hide();
         
         
         // Masquer un élément 
         sym.$("Text1").hide();
         sym.$("Text2").show();
         sym.$("Text3").hide();
         
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(1500);
         
         
         }
         
         sym.$("rendu")[0].currentTime = 4.75;
         if (sym.$("rendu")[0].paused) {
         	sym.$("rendu")[0].play();
         }
         
         sym.$("rendu")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("rendu")[0].currentTime >=7){
           sym.$("rendu")[0].pause();}
         
         
           }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${_1}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         
         sym.$("b1").show();
         sym.$("b2").hide();
         sym.$("b3").hide();
         sym.$("pointer").hide();
         sym.$("Rectangle2").hide();
         
         
         sym.$("Text1").show();
         sym.$("Text2").hide();
         sym.$("Text3").hide();
         
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(250);
         
         
         }
         
         
         
         sym.$("rendu")[0].currentTime = 0.1;
         if (sym.$("rendu")[0].paused) {
         	sym.$("rendu")[0].play();
         }
         
         sym.$("rendu")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("rendu")[0].currentTime >=4.25){
           sym.$("rendu")[0].pause();}
         
         
           }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
         play=1;
         
         
         
         sym.$("Text2").hide();
         sym.$("Text3").hide();
         sym.$("Text1").hide();
         
         
         
         x=-1;
         
         // Masquer un élément 
         sym.$("b1").hide();
         
         // Masquer un élément 
         sym.$("b2").hide();
         
         // Masquer un élément 
         sym.$("b3").hide();
         
         
         
         
         sym.stop(0);
         

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
         
         sym.$("b1").show();
         sym.$("b2").hide();
         sym.$("b3").hide();
         sym.$("pointer").hide();
         sym.$("Rectangle3").show();
         sym.$("Text1").show();
         sym.$("Text2").hide();
         sym.$("Text3").hide();
         
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(250);
         }
         
         
         
         
         
         sym.$("rendu")[0].currentTime = 0.1;
         if (sym.$("rendu")[0].paused) {
         	sym.$("rendu")[0].play();
         }
         
         sym.$("rendu")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("rendu")[0].currentTime >=4.25){
           sym.$("rendu")[0].pause();}
         
         
           }

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