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
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         sym.$('Radio1').html("<input type='radio' id='check1' name='choix' value='1'"  + "style='background-color:#eee;width:40px; height:40px;'>");
         sym.$('Radio2').html("<input type='radio' id='check2' name='choix' value='1'"  + "style='background-color:#eee;width:40px; height:40px;'>");
         sym.$('Radio3').html("<input type='radio' id='check3' name='choix' value='1'"  + "style='background-color:#eee;width:40px; height:40px;'>");
         sym.$("Text2").hide();
         sym.$("Explication").hide();
         sym.$("star_bot1").hide();
         
         
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         
         
         if ((check1.checked==false)||(check2.checked==false)||(check3.checked==false))
         {
         sym.$("Text2").show();
         }
         
         if(check1.checked==true)
         {
         sym.$("Text2").hide();
         sym.$('RoundRect').css("border-color","green") ;
         sym.$("Explication").fadeIn(500);
         sym.$("star_bot_op").hide();
         sym.$("star_bot1").show();
         check1.disabled = true;
         check2.disabled = true;
         check3.disabled = true;
         parent.Reveal.Knowledge[1]=1;
         }
         else if (check2.checked==true)
         {
         sym.$("Text2").hide();
         sym.$('RoundRect2').css("border-color","Red") ;
         sym.$('RoundRect').css("border-color","green") ;
         sym.$("Explication").fadeIn(500);
         check1.disabled = true;
         check2.disabled = true;
         check3.disabled = true;
         }
         else if (check3.checked==true)
         {
         sym.$("Text2").hide();
         sym.$('RoundRect3').css("border-color","Red") ;
         sym.$('RoundRect').css("border-color","green") ;
         sym.$("Explication").fadeIn(500);
         check1.disabled = true;
         check2.disabled = true;
         check3.disabled = true;
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

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

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