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
         sym.$("pause").hide();
         sym.$("play1").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${full_screen}", "click", function(sym, e) {
         	// play=1 au stage
         	sym.$("cache_cadre").show();
         	if (play==1) {
         	sym.$("a7_discution2").fadeOut(300);
         	sym.$("a7_discution").fadeOut(300);
         	sym.$("Text2").fadeOut(300);
         	sym.$("play_boutton").fadeOut(300);
         	sym.play(5000);
         	play=-1;
         	//Atef !!!! inserer un declencheur sym.stop()a la fin du symbole
         	sym.$("demo").fadeOut(300);
         	
         	
         	if (sym.$("larobotique3")[0].paused) {
         		sym.$("larobotique3")[0].play();
         	
         	}
         	
         	sym.$("larobotique3")[0].ontimeupdate = function() {videoPause()};
         	
         	function videoPause() {
         	
         	  if (sym.$("larobotique3")[0].currentTime >=sym.$("larobotique3")[0].duration){
         	
         	  sym.$("larobotique3")[0].pause();
         	if (play!=1){
         	sym.playReverse(5500);}
         	//Atef !!!! inserer un declencheur sym.stop() au debut du symbole
         	  sym.$("a7_discution2").fadeIn(500);
         	  sym.$("a7_discution").fadeIn(500);
         	  sym.$("larobotique3")[0].currentTime = 0;
         	  play=1;
         	  sym.$("Text2").fadeIn(500);
         	
         	
         	  }
         	
         	}
         	}
         	else 
         	{
         	
         	play=1;
         	sym.playReverse(5500);
         	sym.$("Text2").fadeIn(800);
         	sym.$("a7_discution2").fadeIn(800);
         	sym.$("a7_discution").fadeIn(800);
         	};
         	
         	
         	
         	
         	
         	

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

      

      

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${play_boutton}", "click", function(sym, e) {
         
         // Masquer un élément 
         sym.$("play_boutton").hide();
         
         sym.$("larobotique3")[0].currentTime = 0;
         if (sym.$("larobotique3")[0].paused) {
         	sym.$("larobotique3")[0].play();
         
         }
         
         sym.$("larobotique3")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("larobotique3")[0].currentTime >=sym.$("larobotique3")[0].duration){
           sym.$("larobotique3")[0].pause();
           sym.$("larobotique3").show();
           sym.$("larobotique3")[0].currentTime = 0.5;
           }
         
         
           }
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${larobotique3}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         if (sym.$("larobotique3")[0].paused) {
         	sym.$("larobotique3")[0].play();
         	sym.$("play_boutton").hide();
         }else {
         sym.$("larobotique3")[0].pause();
         }
         sym.$("larobotique3")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("larobotique3")[0].currentTime >=sym.$("larobotique3")[0].duration){
           sym.$("larobotique3")[0].pause();
           	sym.$("play_boutton").fadeIn(300);
           sym.$("larobotique3").show();
           sym.$("larobotique3")[0].currentTime = 0.5;
           }
         
         
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
   
   //Edge symbol: 'TITRE_2'
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

   })("TITRE_2");
   //Edge symbol end:'TITRE_2'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");