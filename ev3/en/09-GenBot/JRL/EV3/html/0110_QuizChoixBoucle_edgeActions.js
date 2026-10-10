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
      
      
      

      Symbol.bindElementAction(compId, symbolName, "${Rectangle2Copy3}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Rectangle2Copy}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         sym.$("Rectangle2Copy").data('clicked', false);
         sym.$('Rectangle2Copy3').data('clicked',false);
         
         sym.$("Text2").hide();
         sym.$("Explication").hide();
         sym.$("boutonCopy").hide();
         sym.$("star_bot1").hide();
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         
         if ((sym.$('Rectangle2Copy').data('clicked'))||(sym.$('Rectangle2Copy3').data('clicked'))){
         sym.$("Text2").hide();
         sym.$("bouton").unbind();
         
         if ((sym.$('Rectangle2Copy').data('clicked')==true)&&(sym.$('Rectangle2Copy3').data('clicked')==false)){
         
         sym.$('Rectangle2Copy').css("background-color","green")  ;
         sym.$("Text2").hide();
         sym.$("bouton").hide();
         sym.$("boutonCopy").show();
         sym.$("Rectangle2Copy3").unbind("click");
         sym.$("Rectangle2Copy").unbind("click");
         sym.$("programme22").unbind("click");
         sym.$("A5_programme1").unbind("click");
         sym.$("star_bot1").show();
         parent.Reveal.Coding[1]=1;
         
         }
         else
         {
         sym.$('Rectangle2Copy').css("background-color","green")  ;
         if(sym.$('Rectangle2Copy3').data('clicked')) { 
         sym.$("Rectangle2Copy3").css("background-color","red")};
         sym.$("Text2").hide();
         sym.$("bouton").hide();
         sym.$("boutonCopy").show();
         sym.$("Rectangle2Copy3").unbind("click");
         sym.$("Rectangle2Copy").unbind("click");
         sym.$("programme22").unbind("click");
         sym.$("A5_programme1").unbind("click");
         
         }
         }
         else
         {sym.$("Text2").show();}
         parent.Reveal.TotalCo();
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

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${boutonCopy}", "click", function(sym, e) {
         
         sym.$("Explication").fadeIn(600);
         sym.$("Rectangle2Copy").css("background-color","#c0c0c0")
         sym.$("Rectangle2Copy3").css("background-color","#c0c0c0")
         sym.$("Rectangle2Copy3").fadeTo(500,0.1);
         sym.$("programme22").fadeTo(500,0.05)
         sym.$("A5_programme1").fadeTo(500,0.05)
         sym.$("Rectangle2Copy").fadeTo(500,0.05)

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGECopy").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${programme22}", "click", function(sym, e) {
         // insert code for mouse click here
         
         
         if(sym.$('Rectangle2Copy').data('clicked')) { 
         sym.$("Rectangle2Copy").css("background-color","##c0c0c0")
         sym.$('Rectangle2Copy').data('clicked',false);
         }
         else
         {
         sym.$("Rectangle2Copy").css("background-color","#4b4b4b")
         sym.$('Rectangle2Copy').data('clicked',true);
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A5_programme1}", "click", function(sym, e) {
         
         // insert code for mouse click here
         
         
         if(sym.$('Rectangle2Copy3').data('clicked')) { 
         sym.$("Rectangle2Copy3").css("background-color","##c0c0c0")
         sym.$('Rectangle2Copy3').data('clicked',false);
         }
         else
         {
         sym.$("Rectangle2Copy3").css("background-color","#4b4b4b")
         sym.$('Rectangle2Copy3').data('clicked',true);
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