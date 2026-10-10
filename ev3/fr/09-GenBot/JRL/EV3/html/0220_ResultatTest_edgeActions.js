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
         
         sym.$('resultat').hide();
         
         
         
         sym.$('Text1').html("<output type='text' id='input1' " + "  tabindex='1' " +
         "   letter-spacing: 2px; color: red; padding-left:0px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text2').html("<output type='text' id='input2' " + "  tabindex='1' " +
         "   letter-spacing: 2px; color: red; padding-left:0px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text3').html("<output type='text' id='input3' " + "  tabindex='1' " +
         "   letter-spacing: 2px; color: red; padding-left:0px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text4').html("<output type='text' id='input4' " + "  tabindex='1' " +
         "   letter-spacing: 2px; color: red; padding-left:0px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text5').html("<output type='text' id='input5' " + "  tabindex='1' " +
         "   letter-spacing: 2px; color: red; padding-left:0px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text6').html("<output type='text' id='input6' " + "  tabindex='1' " +
         "   letter-spacing: 2px; color: red; padding-left:0px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text7').html("<output type='text' id='input7' " + "  tabindex='1' " +
         "   letter-spacing: 2px; color: red; padding-left:0px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text8').html("<output type='text' id='input8' " + "  tabindex='1' " +
         "   letter-spacing: 2px; color: red; padding-left:0px; text-align:left; font-size:70px;font-family: MagistralC;'>");

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

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         
         
         reponse = new Array(0,0,0,0,0,0);
         j=0;
         for (i=0;i<8;i++)
         {j=j+1;
         reponse[i]=parent.Reveal.Mesure[i];
         sym.$('Text'+j).html(reponse[i]).css({"font-weight": "bold","color": "red", "text-align":"center","font-size":"65px","font-family": "MagistralC"});
         
         }
         
         
         x=2.85;
         pmin=0;
         min = Math.abs(x-reponse[0]);
         for (i=1;i<4;i++)
         {
         if (Math.abs(x-reponse[i])<min){
         min=Math.abs(x-reponse[i]);
         pmin=i;
         }
         }
         sym.$('resultat').fadeIn(500);
         sym.$("nombre").html(x);
         sym.$("proche").html(reponse[pmin]);

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

      Symbol.bindElementAction(compId, symbolName, "document", "keyup", function(sym, e) {
         // insérer le code à exécuter lorsque vous cessez d'appuyer sur une touche.
         // La variable “e.which” indique le code associé à la touche sur laquelle vous avez appuyé, par exemple 13 = entrée
         if (e.which == 13) {
         sym.$("valider").click();
         	// exécuter une action
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

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");