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
          sym.$("pause").hide();
           sym.$("play1").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${play_bouttonCopy}", "click", function(sym, e) {
         
         // Masquer un élément 
         sym.$("play_bouttonCopy").fadeOut(500);
         
         sym.$("quiz_frein")[0].currentTime = 0;
         if (sym.$("quiz_frein")[0].paused) {
         setTimeout(function(){ sym.$("quiz_frein")[0].play(); }, 0);
                                     clearTimeout();
         
         }
         
         sym.$("quiz_frein")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("quiz_frein")[0].currentTime >=sym.$("quiz_frein")[0].duration){
           sym.$("quiz_frein")[0].pause();
           sym.$("quiz_frein").show();
           sym.$("quiz_frein")[0].currentTime = 0.0;
             sym.$("play_bouttonCopy").fadeIn(500);
           }
         
         
           }
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${quiz_frein}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insert code for mouse click here
         
         // Play a video track 
         sym.$("XiaoYing_Video_1586782967049")[0].play();

      });
      //Edge binding end

   })("stage");
   //Edge symbol end:'stage'

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

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");