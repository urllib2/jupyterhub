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

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${play_boutton}", "click", function(sym, e) {
         
         // Masquer un élément 
         sym.$("play_boutton").hide();
         
         sym.$("larobotique3")[0].currentTime = 0;
         if ((sym.$("larobotique3")[0].paused)&&(sym.$("portvue")[0].paused)) 
         {
         	sym.$("larobotique3")[0].play();
         	sym.$("portvue")[0].play();
         }
         
         sym.$("larobotique3")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("larobotique3")[0].currentTime >=sym.$("larobotique3")[0].duration){
           sym.$("larobotique3")[0].pause();
           sym.$("larobotique3").show();
           sym.$("larobotique3")[0].currentTime = 0.5;
           sym.$("play_boutton").fadeIn(500);
           }
         
         
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

      

      

      Symbol.bindElementAction(compId, symbolName, "${larobotique3}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         if ((sym.$("larobotique3")[0].paused) &&(sym.$("portvue")[0].paused))
         {
         	sym.$("larobotique3")[0].play();
         	sym.$("portvue")[0].play();
         	sym.$("play_boutton").hide();
         }
         else 
         {
         sym.$("larobotique3")[0].pause();
         sym.$("portvue")[0].pause();
         }

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
   
   //Edge symbol: 'TITRE_2_1'
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

   })("TITRE_2_1");
   //Edge symbol end:'TITRE_2_1'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");