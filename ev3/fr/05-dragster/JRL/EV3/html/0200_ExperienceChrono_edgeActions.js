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
          sym.$('Text1').html("<input type='text' id='input1' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:55px; letter-spacing: 2px; color: blue; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${full_screen}", "click", function(sym, e) {
         	// play=1 au stage
         	sym.$("cache_cadre").show();
         	if (play==1) {
         	
         	sym.$("intro").fadeOut(500);
         	sym.$("Message").fadeOut(200);
         	sym.play(5000);
         	play=-1;
         	//Atef !!!! inserer un declencheur sym.stop()a la fin du symbole
         	sym.$("play_boutton").fadeOut(500);
         	
         	
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
         	  sym.$("play_boutton").fadeIn(500);
         	  sym.$("larobotique3")[0].currentTime = 0;
         	  play=1;
         	  sym.$("intro").fadeIn(500);
         	 sym.$("Message").fadeIn(500);
         	  }
         	
         	}
         	}
         	else 
         	{
         	
         	play=1;
         	sym.playReverse(5500);
         	sym.$("intro").fadeIn(500);
         	 sym.$("Message").fadeIn(500);
         	};
         	
         	
         	
         	
         	
         	

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

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         
         
         
         
         x=0;
         
         
             if(input1.value>0)
             {
              input1.disabled = true; 
              sym.$("Text3").click (false);
              sym.$("GREY").click (false);
         		x=input1.value;
         		sym.$("Message").css("color", "#05a000");
         		sym.$("Message").html("Valeur Enregistré");
         		parent.Reveal.Mesure[1]=x;
              }
         	else 
         		{
         		sym.$("Message").css("color", "red");
         		sym.$("Message").html("Verifier votre valeur");
         		}
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

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