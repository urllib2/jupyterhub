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
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${intergace}", "click", function(sym, e) {
         
         // insert code for mouse click here
         
         
         
         if (sym.$("_3D")[0].paused) {
         	sym.$("_3D")[0].play();
         sym.$("play").fadeOut(200);
         }
         else {
         sym.$("_3D")[0].pause();
         }
         sym.$("_3D")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("_3D")[0].currentTime >=sym.$("_3D")[0].duration){
           sym.$("_3D")[0].pause();
           sym.$("_3D")[0].currentTime = 0;
           sym.$("play").fadeIn(200);
           }
         
         
           }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${play}", "click", function(sym, e) {
         
         // insert code for mouse click here
         
         
         
         if (sym.$("_3D")[0].paused) {
         	sym.$("_3D")[0].play();
         sym.$("play").fadeOut(200);
         }
         else {
         sym.$("_3D")[0].pause();
         }
         sym.$("_3D")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("_3D")[0].currentTime >=sym.$("_3D")[0].duration){
           sym.$("_3D")[0].pause();
           sym.$("_3D")[0].currentTime = 0;
           sym.$("play").fadeIn(200);
           }
         
         
           }

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