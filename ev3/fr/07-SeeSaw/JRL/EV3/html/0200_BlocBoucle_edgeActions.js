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
      
      
      Symbol.bindElementAction(compId, symbolName, "${blo6}", "click", function(sym, e) {
         sym.$("Text2").hide();// insérer le code du clic de souris ici
         
         
         
         if(sym.$('Rectangle2').data('clicked')) { 
         sym.$("Rectangle2").css("background-color","##c0c0c0")
         sym.$('Rectangle2').data('clicked',false);
         }
         else
         {
         sym.$("Rectangle2").css("background-color","#4b4b4b")
         sym.$('Rectangle2').data('clicked',true);
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Rectangle2Copy}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Rectangle2Copy3}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Rectangle2Copy4}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Rectangle2Copy5}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         

      });
      //Edge binding end

      

      

      Symbol.bindElementAction(compId, symbolName, "${blo2}", "click", function(sym, e) {
         sym.$("Text2").hide();
         
         
         
         
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

      

      Symbol.bindElementAction(compId, symbolName, "${blo5}", "click", function(sym, e) {
         sym.$("Text2").hide();
         
         if(sym.$('Rectangle2Copy5').data('clicked')) { 
         sym.$("Rectangle2Copy5").css("background-color","##c0c0c0")
         sym.$('Rectangle2Copy5').data('clicked',false);
         }
         else
         {
         sym.$("Rectangle2Copy5").css("background-color","#4b4b4b")
         sym.$('Rectangle2Copy5').data('clicked',true);
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${blo4}", "click", function(sym, e) {
         sym.$("Text2").hide();
         if(sym.$('Rectangle2Copy4').data('clicked')) { 
         sym.$("Rectangle2Copy4").css("background-color","##c0c0c0")
         sym.$('Rectangle2Copy4').data('clicked',false);
         }
         else
         {
         sym.$("Rectangle2Copy4").css("background-color","#4b4b4b")
         sym.$('Rectangle2Copy4').data('clicked',true);
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${blo1}", "click", function(sym, e) {
         sym.$("Text2").hide();
         
         
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
         
         
         
         
         // Accéder à une position de lecture précise (en secondes) 
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         
         
         //Si au moins un bloc est coché "cocher une réponse" est cachée
         if ((sym.$('Rectangle2Copy').data('clicked'))||(sym.$('Rectangle2').data('clicked'))||(sym.$('Rectangle2Copy5').data('clicked'))||(sym.$('Rectangle2Copy4').data('clicked'))||(sym.$('Rectangle2Copy7').data('clicked'))||(sym.$('Rectangle2Copy3').data('clicked'))){
         sym.$("Text2").hide();
         sym.$("blo1").unbind("click");
         sym.$("blo2").unbind("click");
         sym.$("blo3").unbind("click");
         sym.$("blo4").unbind("click");
         sym.$("blo5").unbind("click");
         sym.$("blo6").unbind("click");
         sym.$("Rectangle2").unbind("click");
         sym.$("Rectangle2Copy").unbind("click");
         sym.$("Rectangle2Copy3").unbind("click");
         sym.$("Rectangle2Copy4").unbind("click");
         sym.$("Rectangle2Copy5").unbind("click");
         sym.$("Rectangle2Copy7").unbind("click");
         //Si la case cochée est correcte
         if ((sym.$('Rectangle2Copy5').data('clicked')==false)&&(sym.$('Rectangle2').data('clicked')==false)&&(sym.$('Rectangle2Copy3').data('clicked')==true)&&(sym.$('Rectangle2Copy4').data('clicked')==false)&&(sym.$('Rectangle2Copy7').data('clicked')==false)&&(sym.$('Rectangle2Copy').data('clicked')==false)){
          sym.$('Rectangle2Copy4').css("background-color","green")  ;
         sym.$("Text2").hide();
         sym.$("Gmotor").show();
         sym.getSymbol("Gmotor").play();
         sym.$("bouton").hide();
         
         // Mettre en pause une piste vidéo 
         
         sym.$("large_motor_marker").hide();
         }
         else
         {
         sym.$('Rectangle2Copy3').css("background-color","green")  ;
         if(sym.$('Rectangle2').data('clicked')) { 
         sym.$("Rectangle2").css("background-color","red")};
         if(sym.$('Rectangle2Copy4').data('clicked')) { 
         sym.$("Rectangle2Copy4").css("background-color","red")};
         if(sym.$('Rectangle2Copy5').data('clicked')) { 
         sym.$("Rectangle2Copy5").css("background-color","red")};
         if(sym.$('Rectangle2Copy7').data('clicked')) { 
         sym.$("Rectangle2Copy7").css("background-color","red")};
         //if(sym.$('Rectangle2Copy3').data('clicked')) { 
         //sym.$("Rectangle2Copy3").css("background-color","red")};
         if(sym.$('Rectangle2Copy').data('clicked')) { 
         sym.$("Rectangle2Copy").css("background-color","red")};
         sym.$("Text2").hide();
         sym.$("Gmotor").show();
         sym.getSymbol("Gmotor").play();
         
         sym.$("bouton").hide();
         sym.$("large_motor_marker").hide();
         }
         }
         else
         {sym.$("Text2").show();
         }
         

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
   
   //Edge symbol: 'Gmotor'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         

      });
      //Edge binding end

      

      

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

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 7750, function(sym, e) {
         sym.stop();
         sym.$("Tab1_arrows_etat").hide();
         
         
         // Masquer un élément 
         sym.$("Tab1_arrowsCopy").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows").hide();

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

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4750, function(sym, e) {
         // insérer le code ici
         
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4250, function(sym, e) {
         // insérer le code ici

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

   })("Gmotor");
   //Edge symbol end:'Gmotor'

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