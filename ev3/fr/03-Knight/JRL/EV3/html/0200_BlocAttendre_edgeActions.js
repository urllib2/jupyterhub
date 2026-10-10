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
         sym.$("Rectangle2Copy").data('clicked', false);
         sym.$('Rectangle2').data('clicked',false);
         sym.$('Rectangle2Copy3').data('clicked',false);
         sym.$('Rectangle2Copy4').data('clicked',false);
         sym.$('Rectangle2Copy7').data('clicked',false);
         sym.$('Rectangle2Copy5').data('clicked',false);
         sym.$("Text2").hide();
         
         sym.$("Gmotor").hide();
         sym.$("case").hide();
         
         setTimeout(function(){ sym.$("case").show(); sym.$("progL2").css({'opacity':'0.5'}); }, 3000);
         clearTimeout();
         setTimeout(function(){  sym.$("Gmotor").show();
         sym.getSymbol("Gmotor").play();
         sym.$("progL2").css({'opacity':'0.5'}); }, 5000);
         clearTimeout();
         
         // Accéder à une position de lecture précise (en secondes) 
         sym.$("large_motor_marker")[0].currentTime = 1.5;
         // Lire une piste vidéo 
         sym.$("large_motor_marker")[0].play();
         
         

      });
      //Edge binding end

      

      

      

      

      

      

      

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'sensetive_area_1_2_1'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1000, function(sym, e) {
         // insert code here
         // Play the timeline at a label or specific time. For example:
         // sym.play(500); or sym.play("myLabel");
         
         sym.play(1);

      });
      //Edge binding end

   })("sensetive_area_1_2_1");
   //Edge symbol end:'sensetive_area_1_2_1'

   //=========================================================
   
   //Edge symbol: 'sensetive_area_1_2_2'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1000, function(sym, e) {
         // insert code here
         // Play the timeline at a label or specific time. For example:
         // sym.play(500); or sym.play("myLabel");
         
         sym.play(1);

      });
      //Edge binding end

   })("sensetive_area_1_2_2");
   //Edge symbol end:'sensetive_area_1_2_2'

   //=========================================================
   
   //Edge symbol: 'case'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2000, function(sym, e) {
         // insert code here
         sym.play(0);

      });
      //Edge binding end

   })("case");
   //Edge symbol end:'case'

   //=========================================================
   
   //Edge symbol: 'Gmotor_1'
   (function(symbolName) {   
   
      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_1Copy}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("Tab1_arrowsCopy").show();
         sym.play(6750);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_1Copy}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.play(7251);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_Etat}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         
         sym.$("Tab1_arrows_etat").show();
         
         sym.play(5500);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_Etat}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.play(6001);

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4250, function(sym, e) {
         // insérer le code ici

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4750, function(sym, e) {
         // insérer le code ici
         
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5250, function(sym, e) {
         // insérer le code ici
         sym.stop();
         sym.$("Tab1_arrows_etat").hide();
         
         
         // Masquer un élément 
         sym.$("Tab1_arrowsCopy").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows").hide();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5500, function(sym, e) {
         // insérer le code ici
      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 6000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 6500, function(sym, e) {
         // insérer le code ici
         sym.stop();
         sym.$("Tab1_arrows_etat").hide();
         
         
         // Masquer un élément 
         sym.$("Tab1_arrowsCopy").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows").hide();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 6750, function(sym, e) {
         // insérer le code ici
      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 7250, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 7750, function(sym, e) {
         sym.stop();
         sym.$("Tab1_arrows_etat").hide();
         
         
         // Masquer un élément 
         sym.$("Tab1_arrowsCopy").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows").hide();

      });
      //Edge binding end

   })("Gmotor_1");
   //Edge symbol end:'Gmotor_1'

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