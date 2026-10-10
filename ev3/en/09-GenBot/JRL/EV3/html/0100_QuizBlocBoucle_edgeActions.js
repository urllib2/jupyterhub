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
         sym.$('Text1').html("<input type='number'  id='input1' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='border:0px ; width:180px; height:55px; letter-spacing: 2px; color: black; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
           // Hide an element 
         sym.$("Attention").hide();
         sym.$("Vrai").hide();
         sym.$("Faux").hide();
         sym.$("star_bot1").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         
         if((input1.value.length==0)||(input1.value<0)) 
         {
         sym.$("Attention").show();
         }
         else 
         {
         sym.$("bouton").unbind("click");
         
         
         
         if (input1.value==12) 
         {
         sym.$("Attention").hide();
         sym.$("Vrai").show();
         input1.disabled = true; 
         sym.$("RoundRect").css("border-color","Green")
         parent.Reveal.Coding[0]=1;
         sym.$("star_bot1").show();
         }
         else
         {
         sym.$("Attention").hide();
         sym.$("Faux").show();
         input1.disabled = true;
         sym.$("RoundRect").css("border-color","Red")
         }
         }
         parent.Reveal.TotalCo();
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