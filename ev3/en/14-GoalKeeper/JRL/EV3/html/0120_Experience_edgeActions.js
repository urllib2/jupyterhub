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
         sym.$('Text2').html("<input type='number' id='input1' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='background-color:none; border:0px solid #124a7b; width:200px; height:70px; letter-spacing: 2px; color: blue; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text3').html("<input type='number' id='input2' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='background-color:none; border:0px solid #124a7b; width:200px; height:70px; letter-spacing: 2px; color: blue; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$("mm_relach").hide();
         sym.$("mm_enfonce").hide();
         sym.$("Afficheur1").hide();
         sym.$("Afficheur0").hide();
         sym.$("Text6").hide();
         sym.$("Text7").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
             
             
             if((input1.value.length!=0)&&(input2.value.length!=0))
             {
         sym.$("Group").fadeOut(300);
         sym.$('gk_experience').fadeOut(300);
         sym.$('Group').fadeTo(400,0);
         sym.$('Group3').fadeTo(400,0);
         sym.$('Group2').fadeTo(400,0);
         	 sym.$("mm_relach").fadeIn(300);
         	 sym.$("mm_enfonce").fadeIn(300);
         	 sym.$("Afficheur1").fadeIn(300);
         	 sym.$("Afficheur0").fadeIn(300);
         	 sym.$("Text6").fadeIn(300);
         	 sym.$("Text7").fadeIn(300);
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