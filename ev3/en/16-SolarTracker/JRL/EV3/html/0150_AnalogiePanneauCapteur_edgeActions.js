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
         
         
         sym.$("exemple2").hide();
         sym.$("exemple1").hide();
         x=1;
         a=0;
         b=0;

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         if (x==1) {
         b=1;
         sym.$('exemple2').fadeIn(600);
         sym.$('boutonCopy').fadeOut(300);
         sym.$('Group2').fadeTo(400,0.1);
         sym.$('Group').fadeTo(400,0.1);
         sym.$("capteur")[0].currentTime = 0;
         if (sym.$("capteur")[0].paused) 
         {
         	sym.$("capteur")[0].play();
         
         }
         x=-1
         }
         else if (a==0)
         {b=0;
         x=1;
         sym.$('exemple2').fadeOut(600);
         sym.$('boutonCopy').fadeIn(300);
         sym.$('Group2').fadeTo(400,1);
         sym.$('Group').fadeTo(400,1);
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANG").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "mouseleave", function(sym, e) {
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

      Symbol.bindElementAction(compId, symbolName, "${boutonCopy}", "click", function(sym, e) {
         if (x==1) {
         a=1;
         sym.$('exemple1').fadeIn(600);
         sym.$('bouton').fadeOut(300);
         sym.$('Group2').fadeTo(400,0.1);
         sym.$('Group').fadeTo(400,0.1);
         if (sym.$("panneau")[0].paused) 
         {
         	sym.$("panneau")[0].play();
         
         }
         x=-1
         }
         else if (b==0)
         {a=0;
         x=1;
         sym.$('exemple1').fadeOut(600);
         sym.$('Group2').fadeTo(400,1);
         sym.$('Group').fadeTo(400,1);
         sym.$('bouton').fadeIn(300);
         }

      });
      //Edge binding end

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${boutonCopy}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGECopy").hide();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${boutonCopy}", "mouseover", function(sym, e) {
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "mouseleave", function(sym, e) {
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "mouseover", function(sym, e) {
         sym.$("GREY").hide();
         sym.$("ORANGE").show();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${intergace}", "click", function(sym, e) {
         // insérer le code du clic de souris ici

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${exemple1}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         a=0;
         
         x=1;
         sym.$('exemple1').fadeOut(600);
         sym.$('bouton').fadeIn(300);
         sym.$('Group2').fadeTo(400,1);
         sym.$('Group').fadeTo(400,1);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${exemple2}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         b=0;
         
         x=1;
         sym.$('exemple2').fadeOut(600);
         sym.$('boutonCopy').fadeIn(300);
         sym.$('Group2').fadeTo(400,1);
         sym.$('Group').fadeTo(400,1);

      });
      //Edge binding end

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'TITRE'
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

   })("TITRE");
   //Edge symbol end:'TITRE'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");