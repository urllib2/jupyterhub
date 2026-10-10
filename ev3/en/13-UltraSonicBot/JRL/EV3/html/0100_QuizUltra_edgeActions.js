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
         
         sym.$('Radio1').html("<input type='radio' id='check1' name='choix' value='1'"  + "style='background-color:#eee;width:40px; height:40px;'>");
         sym.$('Radio2').html("<input type='radio' id='check2' name='choix' value='1'"  + "style='background-color:#eee;width:40px; height:40px;'>");
         sym.$('Radio3').html("<input type='radio' id='check3' name='choix' value='1'"  + "style='background-color:#eee;width:40px; height:40px;'>");
         sym.$('Radio4').html("<input type='radio' id='check4' name='choix' value='1'"  + "style='background-color:#eee;width:40px; height:40px;'>");
         sym.$("star_bot1").hide();
         sym.$("star_bot2").hide();
         sym.$("star_bot3").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         //Aucune Reponse selectionné
         if((check1.checked==false)&&(check2.checked==false)&&(check3.checked==false)&&(check4.checked==false))
         {
         sym.$("Message").html("Check an answer!!");
         }
         //Reponse correct selectioné
         if (check2.checked==true) 
         {
         sym.$("Message").hide();
         sym.$("star_bot1").show();
         sym.$("star_bot2").show();
         sym.$("star_bot3").show();
         sym.$("Rectangle2").css("border-color","Green")
         sym.$("Rectangle1").css("border-color","Red")
         sym.$("Rectangle3").css("border-color","Red")
         sym.$("Rectangle4").css("border-color","Red")
         sym.$("valider").unbind();
         check1.disabled = true;
         check2.disabled = true;
         check3.disabled = true;
         check4.disabled = true;
         parent.Reveal.Coding[0]=3;
         }
         //Reponse Fausse selectionné
         else if((check1.checked==true) ||(check3.checked==true) ||(check4.checked==true)) 
         {
         sym.$("Message").hide();
         sym.$("Rectangle2").css("border-color","Green")
         sym.$("Rectangle1").css("border-color","Red")
         sym.$("Rectangle3").css("border-color","Red")
         sym.$("Rectangle4").css("border-color","Red")
         sym.$("valider").unbind();
         check1.disabled = true;
         check2.disabled = true;
         check3.disabled = true;
         check4.disabled = true;
         }
         
         parent.Reveal.TotalCo();
         parent.Reveal.updateStarsNumber();
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         sym.$("ORANGE").hide();
         sym.$("GREY").show();

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

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5250, function(sym, e) {
         // insérer le code ic
         sym.stop();

      });
      //Edge binding end

   })("TITRE_2");
   //Edge symbol end:'TITRE_2'

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