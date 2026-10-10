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
         
         play=1;
         sym.stop(0);
         sym.$('car1').hide();
         sym.$('medical1').hide();
         sym.$('tractor1').hide();
         sym.$('research1').hide();
         sym.$('domestic1').hide();
         sym.$('arm1').hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${car}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         // Show an element 
         sym.$("car").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("car1").show();
         sym.$("Text1").fadeOut(50);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${car}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("car").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("car1").hide();
         sym.$("Text1").fadeIn(500);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${car1}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Show an element 
         sym.$("car").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("car1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${car1}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("car").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("car1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${car1}", "click", function(sym, e) {
         
         // Replay a video track from the beginning, regardless of current playing state 
         sym.$("Oz_test")[0].currentTime = 52;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=87){
           sym.$("Oz_test")[0].pause();}
         
         
           }
         
         sym.$("play_boutton").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${car}", "click", function(sym, e) {
         
         // Replay a video track from the beginning, regardless of current playing state 
         sym.$("Oz_test")[0].currentTime = 52;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=91){
           sym.$("Oz_test")[0].pause();}
         
         
           }
         
         sym.$("play_boutton").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         // Show an element 
         sym.$("arm").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("arm1").show();
         sym.$("Text").fadeOut(50);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("arm").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("arm1").hide();
         sym.$("Text").fadeIn(500);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm}", "click", function(sym, e) {
         
         // Replay a video track from the beginning, regardless of current playing state 
         sym.$("Oz_test")[0].currentTime = 30;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=50){
           sym.$("Oz_test")[0].pause();}
         
         
           }
         sym.$("play_boutton").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm1}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Show an element 
         sym.$("car").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("car1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm1}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("arm").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("arm1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm1}", "click", function(sym, e) {
         
         // Replay a video track from the beginning, regardless of current playing state 
         sym.$("Oz_test")[0].currentTime = 30;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=50){
           sym.$("Oz_test")[0].pause();}
         
         
           }
         sym.$("play_boutton").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${tractor}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         // Show an element 
         sym.$("tractor").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("tractor1").show();
         sym.$("Text2").fadeOut(50);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${tractor}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         sym.$("tractor").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("tractor1").hide();
         sym.$("Text2").fadeIn(500);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${tractor}", "click", function(sym, e) {
         
         // Replay a video track from the beginning, regardless of current playing state 
         sym.$("Oz_test")[0].currentTime = 0;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=29){
           sym.$("Oz_test")[0].pause();}
         
         
           }
         sym.$("play_boutton").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${tractor1}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Show an element 
         sym.$("car").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("car1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${tractor1}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("car").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("car1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${tractor1}", "click", function(sym, e) {
         // insert code for mouse click here
         sym.$("Oz_test")[0].currentTime = 0;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=29){
           sym.$("Oz_test")[0].pause();}
         
         
           }
         sym.$("play_boutton").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${play_boutton}", "click", function(sym, e) {
         
         // Masquer un élément 
         sym.$("play_boutton").hide();
         
         sym.$("Oz_test")[0].currentTime = 0;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=sym.$("Oz_test")[0].duration){
           sym.$("Oz_test")[0].pause();
           sym.$("play_boutton").show();
           sym.$("Oz_test")[0].currentTime = 0.5;
             sym.$("play_boutton").fadeIn(500);
           }
         
         
           }
         

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

      

      Symbol.bindElementAction(compId, symbolName, "${Oz_test}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         	sym.$("play_boutton").hide();
         }else {
         sym.$("Oz_test")[0].pause();
         }
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=sym.$("Oz_test")[0].duration){
           sym.$("Oz_test")[0].pause();
           	sym.$("play_boutton").fadeIn(300);
           sym.$("Oz_test").show();
           sym.$("Oz_test")[0].currentTime = 0.5;
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