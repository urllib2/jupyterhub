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
      
      
      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 500, function(sym, e) {
         // insérer le code ici
         // Définir un sélecteur permettant de lire ou de mettre en pause le scénario de symbole 
         
         sym.stop();

      });
      //Edge binding end

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1750, function(sym, e) {
         // insérer le code ici
         
         	sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2250, function(sym, e) {
         // insérer le code ici
         sym.stop();
         sym.$("Tab1_arrows_etat").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_programme").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_projet").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrowsCopy").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows").hide();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1250, function(sym, e) {
         // insérer le code ici
         

      });
      //Edge binding end

      

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1000, function(sym, e) {
         // insérer le code ici
         sym.stop();
         sym.$("Tab1_arrows_etat").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_programme").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_projet").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrowsCopy").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_projet}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         
         
         // Masquer un élément 
         sym.$("Tab1_arrows_projet").show();
         sym.play(2500);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_projet}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.play(3001);

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2500, function(sym, e) {
         // insérer le code ici
         

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 3000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 3500, function(sym, e) {
         // insérer le code ici
         
         	sym.stop();
         sym.$("Tab1_arrows_etat").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_programme").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_projet").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrowsCopy").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_programme}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         
         // Masquer un élément 
         sym.$("Tab1_arrows_programme").show();
         sym.play(4000);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_programme}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.play(4501);

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4000, function(sym, e) {
         // insérer le code ici
         
         

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4500, function(sym, e) {
         // insérer le code ici
         
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5000, function(sym, e) {
         // insérer le code ici
         
         	sym.stop();
         sym.$("Tab1_arrows_etat").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_programme").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_projet").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrowsCopy").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows").hide();

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
         sym.$("Tab1_arrows_programme").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_projet").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrowsCopy").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         // Masquer un élément 
         sym.$("Tab1_arrows_etat").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_programme").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_projet").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrowsCopy").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows").hide();
         
         sym.$("Group").hide();
         sym.$("interface2").hide();
         sym.$("validerCopy2").hide();
         sym.$("Text1").hide();
         sym.$("Text3").hide();
         sym.$("Text4").hide();sym.$("Text5").hide();
         sym.$("case").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_1Copy}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("Tab1_arrows").show();
         sym.play(0)

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_1Copy}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.play(501);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_1}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("Tab1_arrowsCopy").show();
         sym.play(1250);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${sensetive_area_1}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.play(1751);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         
         sym.$("TextCopy").fadeOut(500);
         sym.$("valider").fadeOut(500);
         sym.$("Group").fadeIn(1000);
         sym.$("validerCopy2").delay(6000).fadeIn(500);
         sym.$("Tab1_arrows_etat").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_programme").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows_projet").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrowsCopy").hide();
         
         // Masquer un élément 
         sym.$("Tab1_arrows").hide();
         
         // Masquer un élément 
         sym.$("Text2").hide();

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

      Symbol.bindElementAction(compId, symbolName, "${validerCopy2}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         
         sym.$("interface2").fadeIn(500);
         sym.$("sensetive_area_Etat").fadeOut(200);
         sym.$("sensetive_area_1").fadeOut(200);
         sym.$("sensetive_area_1Copy").fadeOut(200);
         sym.$("sensetive_area_projet").fadeOut(200);
         sym.$("sensetive_area_programme").fadeOut(200);
         
         sym.$('Text1').delay(2000).fadeIn(500)  ;
         sym.$('Text1').delay(7500).fadeOut(200)  ;
         sym.$('Text3').delay(11000).fadeIn(500)  ;
         sym.$('Text3').delay(4000).fadeOut(200)  ;
         sym.$('Text4').delay(20000).fadeIn(500)  ;
         sym.$('case').delay(21000).fadeIn(500)  ;
         sym.$('Text4').delay(5000).fadeOut(200)  ;
         sym.$('case').delay(4000).fadeOut(200)  ;
         sym.$('Text5').delay(31500).fadeIn(500)  ;
         sym.$('Text5').delay(9000).fadeOut(200)  ;
         // Lire une piste vidéo 
         sym.$("interface2")[0].play();
         
         sym.$("interface2")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("interface2")[0].currentTime >=sym.$("interface2")[0].duration){
           sym.$("interface2").fadeOut(200);
           sym.$("sensetive_area_Etat").fadeIn(200);
         sym.$("sensetive_area_1").fadeIn(200);
         sym.$("sensetive_area_1Copy").fadeIn(200);
         sym.$("sensetive_area_projet").fadeIn(200);
         sym.$("sensetive_area_programme").fadeIn(200);
         
           }
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${validerCopy2}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy2").hide();
         sym.$("ORANGECopy2").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${validerCopy2}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy2").show();
         sym.$("ORANGECopy2").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Group}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         

      });
      //Edge binding end

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'sensetive_area_1'
   (function(symbolName) {   
   
      

   })("sensetive_area_1");
   //Edge symbol end:'sensetive_area_1'

   //=========================================================
   
   //Edge symbol: 'sensetive_area_1_1'
   (function(symbolName) {   
   
      

   })("sensetive_area_1_1");
   //Edge symbol end:'sensetive_area_1_1'

   //=========================================================
   
   //Edge symbol: 'sensetive_area_1_2'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1000, function(sym, e) {
         // insert code here
         // Play the timeline at a label or specific time. For example:
         // sym.play(500); or sym.play("myLabel");
         
         sym.play(1);

      });
      //Edge binding end

   })("sensetive_area_1_2");
   //Edge symbol end:'sensetive_area_1_2'

   //=========================================================
   
   //Edge symbol: 'sensetive_area_1_3'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1000, function(sym, e) {
         // insérer le code ici
         sym.play(1);

      });
      //Edge binding end

   })("sensetive_area_1_3");
   //Edge symbol end:'sensetive_area_1_3'

   //=========================================================
   
   //Edge symbol: 'sensetive_area_1_4'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1000, function(sym, e) {
         // insert code here
         // Play the timeline at a label or specific time. For example:
         // sym.play(500); or sym.play("myLabel");
         sym.play(1);

      });
      //Edge binding end

   })("sensetive_area_1_4");
   //Edge symbol end:'sensetive_area_1_4'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

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

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");