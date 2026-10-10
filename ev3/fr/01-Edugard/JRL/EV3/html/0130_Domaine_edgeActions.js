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
         sym.$("Oz_test")[0].currentTime = 99;
         //sym.$("Oz_test")[0].stop();
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
         sym.$("Oz_test")[0].currentTime = 21;
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

      Symbol.bindElementAction(compId, symbolName, "${car}", "click", function(sym, e) {
         
         // Replay a video track from the beginning, regardless of current playing state 
         sym.$("Oz_test")[0].currentTime = 21;
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
         sym.$("Oz_test")[0].currentTime = 0;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=19.5){
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
         sym.$("Oz_test")[0].currentTime = 0;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=19.5){
           sym.$("Oz_test")[0].pause();}
         
         
           }
         sym.$("play_boutton").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         // Show an element 
         sym.$("domestic").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("domestic1").show();
         sym.$("Text4").fadeOut(50);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("domestic").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("domestic1").hide();
         sym.$("Text4").fadeIn(500);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic}", "click", function(sym, e) {
         
         // Replay a video track from the beginning, regardless of current playing state 
         sym.$("Oz_test")[0].currentTime = 101;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=118){
           sym.$("Oz_test")[0].currentTime=101;
           sym.$("play_boutton").fadeIn(500);
           sym.$("Oz_test")[0].pause();
         
           }
         
         
           }
         sym.$("play_boutton").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic1}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Show an element 
         sym.$("domestic").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("domestic1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic1}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("domestic").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("domestic1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic1}", "click", function(sym, e) {
         
         // Replay a video track from the beginning, regardless of current playing state 
         sym.$("Oz_test")[0].currentTime = 101;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=118){
           sym.$("Oz_test")[0].currentTime=101;
           sym.$("play_boutton").fadeIn(500);
           sym.$("Oz_test")[0].pause();
         
           }
         
         
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
         sym.$("Oz_test")[0].currentTime = 83;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=100){
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
           if (sym.$("Oz_test")[0].currentTime >=4){
           sym.$("Oz_test")[0].pause();}
         
         
           }
         sym.$("play_boutton").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         sym.$("research").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("research1").show();
         sym.$("Text3").fadeOut(50);
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("research").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("research1").hide();
         sym.$("Text3").fadeIn(500);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research}", "click", function(sym, e) {
         
         // Replay a video track from the beginning, regardless of current playing state 
         sym.$("Oz_test")[0].currentTime = 51;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=81){
           sym.$("Oz_test")[0].pause();}
         
         
           }
         sym.$("play_boutton").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research1}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Show an element 
         sym.$("research").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("research1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research1}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("research").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("research1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research1}", "click", function(sym, e) {
         
         // Replay a video track from the beginning, regardless of current playing state 
         sym.$("Oz_test")[0].currentTime = 51;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=81){
           sym.$("Oz_test")[0].pause();}
         
         
           }
         sym.$("play_boutton").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${full_screen}", "click", function(sym, e) {
         	// play=1 au stage
         	sym.$("cache_cadre").show();
         	if (play==1) {
         	sym.$('car1').fadeOut(200);
         	sym.$('medical1').fadeOut(200);
         	sym.$('tractor1').fadeOut(200);
         	sym.$('research1').fadeOut(200);
         	sym.$('domestic1').fadeOut(100);
         	sym.$('arm1').fadeOut(200);
         	sym.$('car').fadeOut(200);
         	sym.$('medical').fadeOut(200);
         	sym.$('tractor').fadeOut(200);
         	sym.$('research').fadeOut(200);
         	sym.$('domestic').fadeOut(100);
         	sym.$('arm').fadeOut(200);
         	sym.$("Text").fadeOut(200);
         	sym.$("Text1").fadeOut(200);
         	sym.$("Text2").fadeOut(200);
         	sym.$("Text3").fadeOut(200);
         	sym.$("Text4").fadeOut(100);
         	
         	sym.play(5000);
         	play=-1;
         	//Atef !!!! inserer un declencheur sym.stop()a la fin du symbole
         	sym.$("play_boutton").hide();
         	
         	
         	if (sym.$("Oz_test")[0].paused) {
         		sym.$("Oz_test")[0].play();
         	
         	}
         	
         	sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         	
         	function videoPause() {
         	  if (sym.$("Oz_test")[0].currentTime >=sym.$("Oz_test")[0].duration){
         	  sym.$("Oz_test")[0].pause();
         	
         	sym.playReverse(5500);
         	//Atef !!!! inserer un declencheur sym.stop() au debut du symbole
         	  sym.$("play_boutton").show();
         	  sym.$("Oz_test")[0].currentTime = 0;
         	  play=1;
         	  sym.$('car').fadeIn(500);
         	sym.$('medical').fadeIn(500);
         	sym.$('tractor').fadeIn(500);
         	sym.$('research').fadeIn(500);
         	sym.$('domestic').fadeIn(500);
         	sym.$('arm').fadeIn(500);
         	  sym.$("Text").fadeIn(500);
         	sym.$("Text1").fadeIn(500);
         	sym.$("Text2").fadeIn(500);
         	sym.$("Text3").fadeIn(500);
         	sym.$("Text4").fadeIn(500);
         	
         	  }
         	
         	}
         	}
         	else 
         	{
         	  sym.$('car').fadeIn(500);
         	sym.$('medical').fadeIn(500);
         	sym.$('tractor').fadeIn(500);
         	sym.$('research').fadeIn(500);
         	sym.$('domestic').fadeIn(500);
         	sym.$('arm').fadeIn(500);
         	play=1;
         	sym.playReverse(5500);
         	sym.$("Text").fadeIn(500);
         	sym.$("Text1").fadeIn(500);
         	sym.$("Text2").fadeIn(500);
         	sym.$("Text3").fadeIn(500);
         	sym.$("Text4").fadeIn(500);
         	};
         	
         	
         	
         	
         	
         	

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