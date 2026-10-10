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
         sym.$('button1').html("<input type='checkbox' id='check1' name='choix1' value='1'"  + "style='background-color:#eee;width:50px; height:50px;'>");
         sym.$('button2').html("<input type='checkbox' id='check2' name='choix1' value='1'"  + "style='background-color:#eee;width:50px; height:50px;'>");
         sym.$('button3').html("<input type='checkbox' id='check3' name='choix1' value='1'"  + "style='background-color:#eee;width:50px; height:50px;'>");
         sym.$('button4').html("<input type='checkbox' id='check4' name='choix1' value='1'"  + "style='background-color:#eee;width:50px; height:50px;'>");
         sym.$('button5').html("<input type='checkbox' id='check5' name='choix1' value='1'"  + "style='background-color:#eee;width:50px; height:50px;'>");
         sym.$("star_bot1").hide();
         sym.$("star_bot2").hide();
         sym.$("star_bot3").hide();

      });
      //Edge binding end

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         correct=0;
         //Aucune Reponse selectionné
         if((check1.checked==false)&&(check2.checked==false)&&(check3.checked==false)&&(check4.checked==false)&&(check5.checked==false))
         {
         sym.$("Message").html("Cocher une réponse");
         }
         else
         {
         sym.$("Message").hide();
         if (check1.checked==true)
         {
         sym.$("RoundRect1").css("border-color","Red")
         correct--;
         }
         else
         {
         sym.$("RoundRect1").css("border-color","Green")
         }
         
         if (check2.checked==true)
         {
         correct++;
         sym.$("RoundRect2").css("border-color","Green")
         }
         else
         {
         sym.$("RoundRect2").css("border-color","Red")
         }
         
         if (check3.checked==true)
         {
         correct++;
         sym.$("RoundRect3").css("border-color","Green")
         }
         else
         {
         sym.$("RoundRect3").css("border-color","Red")
         }
         
         if (check4.checked==true)
         {
         correct--;
         sym.$("RoundRect4").css("border-color","Red")
         }
         else
         {
         sym.$("RoundRect4").css("border-color","Green")
         }
         
         if (check5.checked==true)
         {
         correct++;
         sym.$("RoundRect5").css("border-color","Green")
         }
         else
         {
         sym.$("RoundRect5").css("border-color","Red")
         }
         switch (correct)
         {
         case 1 :
         sym.$("star_bot1").show();
         break;
         case 2 :
         sym.$("star_bot1").show();
         sym.$("star_bot2").show();
         break;
         case 3 :
         sym.$("star_bot1").show();
         sym.$("star_bot2").show();
         sym.$("star_bot3").show();
         break;
         }
         sym.$("valider").unbind();
         check1.disabled = true;
         check2.disabled = true;
         check3.disabled = true;
         check4.disabled = true;
         check5.disabled = true;
         }
         if(correct>0)
         {
         parent.Reveal.Knowledge[2]=correct;
         parent.Reveal.TotalKnow();
         parent.Reveal.updateStarsNumber();
         }
         if(correct<0)
         {
         parent.Reveal.Knowledge[2]=0;
         parent.Reveal.TotalKnow();
         parent.Reveal.updateStarsNumber();
         }
         

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