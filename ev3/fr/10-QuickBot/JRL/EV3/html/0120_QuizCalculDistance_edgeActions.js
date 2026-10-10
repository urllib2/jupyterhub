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
         hint=1;
         
         sym.play();

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
         x=43.2
         
             if((input1.value>0)&&(input1.value<1000)){
              input1.disabled = true; 
              sym.$("Text3").click (false);
              sym.$("GREY").click (false);
                 if (input1.value==x){
         				sym.$('star_bot1').show();
         				sym.$("message").css("color", "#05a000");
         				sym.$("message2").html("Bravo ! bonne reponse :)");
         				parent.Reveal.Knowledge[1]=1;
                 }
         			else 
         			{
         				sym.$("message").css("color", "red");
         				sym.$("message").html("C'est pas précis! La valeur = 43.2cm ");
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

      Symbol.bindElementAction(compId, symbolName, "${indices}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         if (hint==1) 
         {
         
         
         sym.play(1000);
         hint=-1;
         
         }
         else 
         {
         
         hint=1;
         sym.playReverse(2000);
         };

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1000, function(sym, e) {
         // insert code here
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2000, function(sym, e) {
         // insert code here
         sym.stop();

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