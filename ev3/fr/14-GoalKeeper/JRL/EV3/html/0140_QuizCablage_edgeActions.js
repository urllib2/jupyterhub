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
      
      
      

      Symbol.bindElementAction(compId, symbolName, "${Rectangle2Copy}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Rectangle2Copy3}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         sym.$("star_bot1").hide();
         sym.$("Rectangle2Copy").data('clicked', false);
         sym.$('Rectangle2').data('clicked',false);
         sym.$('Rectangle2Copy3').data('clicked',false);
         sym.$('Rectangle2Copy4').data('clicked',false);
         sym.$('Rectangle2Copy7').data('clicked',false);
         sym.$('Rectangle2Copy5').data('clicked',false);
         sym.$("Text2").hide();
         sym.$("Gmotor").hide();
         sym.$("Explication").hide();
         
         
         // Accéder à une position de lecture précise (en secondes) 
         sym.$("large_motor_marker")[0].currentTime = 1.5;
         // Lire une piste vidéo 
         sym.$("large_motor_marker")[0].play();
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         
         
         if ((sym.$('Rectangle2Copy').data('clicked'))||(sym.$('Rectangle2Copy3').data('clicked'))){
         sym.$("Text2").hide();
         sym.$("Rectangle2Copy3").unbind("click");
         sym.$("Rectangle2Copy2").unbind("click");
         sym.$("gk_tactilemontage2").unbind("click");
         sym.$("gk-tactilemontage").unbind("click");
         sym.$("Text3").click (false);
         sym.$("GREY").click (false);
         sym.$("Text3").click (false);
         if ((sym.$('Rectangle2Copy').data('clicked')==true)&&(sym.$('Rectangle2Copy3').data('clicked')==false))
         {
         sym.$('Rectangle2Copy').css("background-color","green");
         sym.$("Explication").fadeIn(500);
         sym.$("Text2").hide();
         sym.$("star_bot1").show();
         
         parent.Reveal.Knowledge[1]=1;
         
         }
         else
         {
         sym.$("Rectangle2Copy3").css("background-color","red")};
         sym.$("Text2").hide();
         sym.$("Explication").fadeIn(500);
         
         }
         else 
         {sym.$("Text2").show();
         }
         parent.Reveal.TotalKnow();
         parent.Reveal.updateStarsNumber();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${gk-tactilemontage}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         
         
         
         
         
         if(sym.$('Rectangle2Copy').data('clicked')) { 
         
         sym.$("Rectangle2Copy").css("background-color","#c0c0c0")
         sym.$('Rectangle2Copy').data('clicked',false);
         }
         else
         {
         sym.$('Text2').hide();
         sym.$("Rectangle2Copy").css("background-color","#4b4b4b")
         sym.$('Rectangle2Copy').data('clicked',true);
         sym.$("Rectangle2Copy3").css("background-color","#c0c0c0")
         sym.$('Rectangle2Copy3').data('clicked',false);
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${gk_tactilemontage2}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         
         
         
         if(sym.$('Rectangle2Copy3').data('clicked')) { 
         
         sym.$("Rectangle2Copy3").css("background-color","##c0c0c0")
         sym.$('Rectangle2Copy3').data('clicked',false);
         }
         else
         {
         sym.$('Text2').hide();
         sym.$("Rectangle2Copy3").css("background-color","#4b4b4b")
         sym.$('Rectangle2Copy3').data('clicked',true);
         sym.$("Rectangle2Copy").css("background-color","#c0c0c0")
         sym.$('Rectangle2Copy').data('clicked',false);
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