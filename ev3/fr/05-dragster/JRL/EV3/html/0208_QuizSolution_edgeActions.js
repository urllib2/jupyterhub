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
         sym.$('Text1').html("<input type='text' id='input1' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:55px; letter-spacing: 2px; color: blue; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('star_bot1').hide();
         sym.$('message').hide();
         sym.$('message2').hide();
         sym.$("Group").hide()

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "document", "keyup", function(sym, e) {
         // insérer le code à exécuter lorsque vous cessez d'appuyer sur une touche.
         // La variable “e.which” indique le code associé à la touche sur laquelle vous avez appuyé, par exemple 13 = entrée
         if (e.which == 13) {
         sym.$("valider").click();
         
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         
         x=0;
         
         sym.$("message").fadeIn(500);
         sym.$("message2").fadeIn(500);
         sym.$('message').html('');
         sym.$('message2').html('');
         x=3
         
             if(input1.value>0){
              input1.disabled = true; 
              sym.$("Text3").click (false);
              sym.$("GREY").click (false);
                 if (input1.value==x){
         				sym.$('star_bot1').show();
         				sym.$("Text3").hide();
         				sym.$("message").css("color", "#05a000");
         				sym.$("message").html("Bravo ! bonne reponse :)");
         				parent.Reveal.Knowledge[1]=1;
                 }
         			else 
         			{
         				sym.$("Text3").hide();
         				sym.$("message2").css("color", "red");
         				sym.$("message2").html("C'est pas précis! La valeur = 3 rotations ");
         			}
         
         
         
         	}
         	else{
         	sym.$('message1').show();
         	sym.$('message1').html('Vérifier votre valeur!');
         	}	;	
         
         
         
         setTimeout(function(){ sym.$("message1").hide(); }, 1500);
         clearTimeout();
         parent.Reveal.TotalKnow();
         parent.Reveal.updateStarsNumber();
         
         

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

      Symbol.bindElementAction(compId, symbolName, "${play_boutton}", "click", function(sym, e) {
         sym.$("Group").delay(1000).fadeIn(1000);
         // Masquer un élément 
         sym.$("play_boutton").fadeOut(500);
         
         sym.$("quiz_frein")[0].currentTime = 0;
         if (sym.$("quiz_frein")[0].paused) {
         setTimeout(function(){ sym.$("quiz_frein")[0].play(); }, 1000);
                                     clearTimeout();
         
         }
         
         sym.$("quiz_frein")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("quiz_frein")[0].currentTime >=sym.$("quiz_frein")[0].duration){
           sym.$("quiz_frein")[0].pause();
           sym.$("quiz_frein").show();
           sym.$("quiz_frein")[0].currentTime = 0.0;
             sym.$("play_boutton").fadeIn(500);
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
   
   //Edge symbol: 'TITRE_1_1'
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

   })("TITRE_1_1");
   //Edge symbol end:'TITRE_1_1'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");