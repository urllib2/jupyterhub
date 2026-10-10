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
         sym.$("solu").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${full_screen}", "click", function(sym, e) {
         	// play=1 au stage
         	sym.$("cache_cadre").show();
         	if (play==1) {
         	
         	sym.$("intro").fadeOut(300);
         	
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
         	  sym.$("demo").fadeIn(500);
         	  sym.$("larobotique3")[0].currentTime = 0;
         	  play=1;
         	  sym.$("intro").fadeIn(500);
         	
         	  }
         	
         	}
         	}
         	else 
         	{
         	
         	play=1;
         	sym.playReverse(5500);
         	sym.$("intro").fadeIn(800);
         	sym.$("demo").fadeIn(800);
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

      Symbol.bindElementAction(compId, symbolName, "${demo}", "click", function(sym, e) {
         
         sym.$("Text3").hide();
         sym.$("pause").show();
         sym.$("vid").fadeIn(900);
         //sym.$("vid").css('transform','scale(0.6,0.6)');
         
         
          sym.$("larobotique3")[0].ontimeupdate = function() {videoPause()};
         function videoPause() {
          if (sym.$("larobotique3")[0].currentTime >=sym.$("larobotique3")[0].duration){
          sym.$("play1").show();
           sym.$("pause").hide();
          }
         }
         
         
         if (sym.$("larobotique3")[0].paused) {
          sym.$("larobotique3")[0].play();
          sym.$("pause").show();
           sym.$("play1").hide();
         }
         else{
         sym.$("larobotique3")[0].pause();
         sym.$("play1").show();
          sym.$("pause").hide();
          }
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         
         var y=0;
         var x = 10;
         
         var id = setInterval(frame, 1000);
         function frame() {
           x--;
         if ((x==0)&&(y==0)){
           clearInterval(id);
           sym.$("solu").show(); 
           sym.$("demo").hide();
         sym.$("full_screen").hide();
         
         } 
         
          }
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${solu}", "click", function(sym, e) {
         sym.$("solu").hide(); 
         sym.$("Sol_ex").show();
         sym.$("larobotique3").fadeOut(700);
         sym.$("full_screen").fadeOut(700);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text32}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text32}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY2").hide();
         sym.$("ORANGE2").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text32}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY2").show();
         sym.$("ORANGE2").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY2}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY2}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY2").hide();
         sym.$("ORANGE2").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY2}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY2").show();
         sym.$("ORANGE2").hide();

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