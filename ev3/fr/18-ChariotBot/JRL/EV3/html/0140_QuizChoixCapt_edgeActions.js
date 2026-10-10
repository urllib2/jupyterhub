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
         
           sym.$("solarPanel2").hide();
           sym.$("battery2").hide();
           sym.$("pila2").hide();
           sym.$("legoCable2").hide();
           sym.$("solarPanel2").delay(200).fadeIn(800);
           sym.$("battery2").delay(400).fadeIn(800);
           sym.$("pila2").delay(600).fadeIn(800);
           sym.$("legoCable2").delay(800).fadeIn(800);
         
         
         
         
           // Hide an element 
           sym.$("nombre").hide();
             sym.$("Attention").hide();
           sym.$("message").hide();
             sym.$("alert").hide();
           sym.$("star_bot1").hide();
         
         
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         
         
         if ((check1.checked==false)||(check2.checked==false)||(check3.checked==false)||(check4.checked==false))
         {
         sym.$("Attention").show();
         }
         
         if(check1.checked==true)
         {
         sym.$("Attention").hide();
         sym.$('RoundRect1').css("border-color","red") ;
         sym.$('RoundRect2').css("border-color","red") ;
         sym.$('RoundRect3').css("border-color","green") ;
         sym.$('RoundRect4').css("border-color","red") ;
         check1.disabled = true;
         check2.disabled = true;
         check3.disabled = true;
         check4.disabled = true;
         sym.$("alert").show();
         }
         else if (check2.checked==true)
         {
         sym.$("Attention").hide();
         sym.$('RoundRect1').css("border-color","red") ;
         sym.$('RoundRect2').css("border-color","red") ;
         sym.$('RoundRect3').css("border-color","green") ;
         sym.$('RoundRect4').css("border-color","red") ;
         check1.disabled = true;
         check2.disabled = true;
         check3.disabled = true;
         check4.disabled = true;
         sym.$("alert").show();
         }
         else if (check3.checked==true)
         {
         sym.$("Attention").hide();
         sym.$('RoundRect1').css("border-color","red") ;
         sym.$('RoundRect2').css("border-color","red") ;
         sym.$('RoundRect3').css("border-color","green") ;
         sym.$('RoundRect4').css("border-color","red") ;
         check1.disabled = true;
         check2.disabled = true;
         check3.disabled = true;
         check4.disabled = true;
         sym.$("star_bot1").show();
         sym.$("message").show();
         parent.Reveal.Knowledge[0]=1;
         }
         else if (check4.checked==true)
         {
         sym.$("Attention").hide();
         sym.$('RoundRect1').css("border-color","red") ;
         sym.$('RoundRect2').css("border-color","red") ;
         sym.$('RoundRect3').css("border-color","green") ;
         sym.$('RoundRect4').css("border-color","red") ;
         check1.disabled = true;
         check2.disabled = true;
         check3.disabled = true;
         check4.disabled = true;
         sym.$("alert").show();
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
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGECopy").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGECopy").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Hide an element 
         sym.$("GREY").hide();
         
         // Show an element 
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Hide an element 
         sym.$("ORANGE").hide();
         
         // Show an element 
         sym.$("GREY").show();
         

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