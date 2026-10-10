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
         
         
         
         // Afficher un élément 
         sym.$("Text3").fadeIn(1000);
         
         
         
         // Masquer un élément 
         
         
         // Afficher un élément 
         sym.$("b3").show();
         
         
         
         // Masquer un élément 
         
         sym.$("b4").hide();
         sym.$("Text9").hide();
         sym.$("etape4").hide();
         sym.$("_4Copy").hide();
         
         sym.$("presentBrick")[0].currentTime = 4;
         if (sym.$("presentBrick")[0].paused) {
         	sym.$("presentBrick")[0].play();
         }
         
         sym.$("presentBrick")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("presentBrick")[0].currentTime >=5){
           sym.$("presentBrick")[0].pause();}
         
         
           }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${_2}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         
         
         // Afficher un élément 
         sym.$("b2").show();
         
         
         // Masquer un élément 
         sym.$("Text7").hide();
         sym.$("Text1").hide();
         
         // Afficher un élément 
         sym.$("Text2").fadeIn(1000);
         
         ;
         
         // Masquer un élément 
         sym.$("b3").hide();
         
         
         
         sym.$("b4").hide();
         sym.$("Text9").hide();
         sym.$("etape4").hide();
         sym.$("_3Copy").hide();
         
         sym.$("presentBrick")[0].currentTime = 2;
         if (sym.$("presentBrick")[0].paused) {
         	sym.$("presentBrick")[0].play();
         
         }
         
         sym.$("presentBrick")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("presentBrick")[0].currentTime >=3){
           sym.$("presentBrick")[0].pause();}
         
         
           }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${_1}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // Afficher un élément 
         sym.$("b1").show();
         
         // Afficher un élément 
         sym.$("Text1").fadeIn(1000);
         
         
         sym.$("Text2").hide();
         // Masquer un élément 
         sym.$("b3").hide();
         
         sym.$("pointer").hide();
         
         // Masquer un élément 
         sym.$("Text3").hide();
         
         
         // Masquer un élément 
         sym.$("Text7").hide();
         sym.$("Text9").hide();
         sym.$("Text10").hide();
         // Masquer un élément 
         sym.$("b2").hide();
         sym.$("b4").hide();
         sym.$("Text9").hide();
         
         sym.$("_2Copy").hide();
         
         
         sym.$("presentBrick")[0].currentTime = 0;
         if (sym.$("presentBrick")[0].paused) {
         	sym.$("presentBrick")[0].play();
         }
         
         sym.$("presentBrick")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("presentBrick")[0].currentTime >=1){
           sym.$("presentBrick")[0].pause();}
         
         
           }
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
          
          
          play=1;
          
          
          
         // Masquer un élément 
         sym.$("Text2").hide();
         
         // Masquer un élément 
         sym.$("Text3").hide();
         
         
         
         // Masquer un élément 
         x=-1;
         
         sym.$("Text7").hide();
         sym.$("Text9").hide();
         sym.$("Text1").hide();
         sym.$("Text10").hide();
         // Masquer un élément 
         sym.$("b1").hide();
         
         // Masquer un élément 
         sym.$("b2").hide();
         
         // Masquer un élément 
         sym.$("b3").hide();
         sym.$("b4").hide();
         
         sym.$("b5").hide();
         sym.$("b6").hide();
         sym.stop(0);
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${_4}", "click", function(sym, e) {
         
         sym.$("Text2").hide();
         
         // Masquer un élément 
         
         
         // Afficher un élément 
         sym.$("Text7").fadeIn(1000);
         
         // Masquer un élément 
         
         
         
         
         // Masquer un élément 
         
         
         // Afficher un élément 
         sym.$("b4").show();
         
         // Afficher un élément 
         sym.$("Text3").hide();
         sym.$("_5copy").hide();
         
         
         
         sym.$("presentBrick")[0].currentTime = 6;
         if (sym.$("presentBrick")[0].paused) {
         	sym.$("presentBrick")[0].play();
         }
         
         sym.$("presentBrick")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("presentBrick")[0].currentTime >=7){
           sym.$("presentBrick")[0].pause();}
         
         
           }
         

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

      

      

      Symbol.bindElementAction(compId, symbolName, "${b1}", "click", function(sym, e) {
         
         
         sym.$("Text1").fadeIn(1000);;
         
         // Masquer un élément 
         sym.$("Text10").hide();
         // Masquer un élément 
         sym.$("b3").hide();
         sym.$("b2").hide();
         sym.$("b4").hide();
         sym.$("b5").hide();
         sym.$("b6").hide();
         sym.$("Text9").hide();
         sym.$("Text2").hide();
         sym.$("Text3").hide();
         sym.$("Text7").hide();
         sym.$("_5copy").show();
         sym.$("_6copy").show();
         sym.$("_2Copy").hide();
         sym.$("_3Copy").show();
         sym.$("_4Copy").show();
         
         
         
         sym.$("presentBrick")[0].currentTime = 0.1;
         if (sym.$("presentBrick")[0].paused) {
         	sym.$("presentBrick")[0].play();
         }
         
         sym.$("presentBrick")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("presentBrick")[0].currentTime >=2.24){
           sym.$("presentBrick")[0].pause();}
         
         
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