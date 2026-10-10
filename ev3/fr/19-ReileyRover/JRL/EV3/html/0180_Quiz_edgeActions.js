/***********************
* Actions de compositions pour Adobe Edge Animate
*
* Modifier ce fichier avec précaution, en veillant à conserver 
* les signatures et les commentaires de fonction commençant par « Edge » pour maintenir la 
* possibilité d’interagir avec ces actions depuis Adobe Edge Animate
*
***********************/
(function($, Edge, compId){
var Composition = Edge.Composition, Symbol = Edge.Symbol; // alias pour les classes Edge couramment utilisées

   //Edge symbol: 'stage'
   (function(symbolName) {
      
      
      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         
         
         if ((check1.checked==false)||(check2.checked==false))
         {
         sym.$("Text2").show();
         }
         
         if(check2.checked==true)
         {
         sym.$("Text2").hide();
         sym.$('RoundRect2').css("border-color","green") ;
         sym.$("star_bot1").show();
         check1.disabled = true;
         check2.disabled = true;
         }
         else if (check1.checked==true)
         {
         sym.$("Text2").hide();
         sym.$('RoundRect1').css("border-color","red") ;
         check1.disabled = true;
         check2.disabled = true;
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

      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
         // insert code to be run when the composition is fully loaded here
         
         sym.$('Radio1').html("<input type='radio' id='check1' name='choix' value='1'"  + "style='background-color:#eee;width:40px; height:40px;'>");
         sym.$('Radio2').html("<input type='radio' id='check2' name='choix' value='1'"  + "style='background-color:#eee;width:40px; height:40px;'>");
         sym.$("star_bot1").hide();
         sym.$("star_bot2").hide();
         sym.$("star_bot3").hide();
         sym.$("Text2").hide();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${play_boutton}", "click", function(sym, e) {
         // insert code for mouse click here
         
         // Masquer un élément 
         sym.$("play_boutton").hide();
         
         sym.$("feya9")[0].currentTime = 0;
         if (sym.$("feya9")[0].paused) {
         	sym.$("feya9")[0].play();
         
         }
         
         sym.$("feya9")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("feya9")[0].currentTime >=sym.$("feya9")[0].duration){
           sym.$("feya9")[0].pause();
           sym.$("play_boutton").show();
           sym.$("feya9")[0].currentTime = 0.5;
           }
         
         
           }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${play_boutton2}", "click", function(sym, e) {
         // insert code for mouse click here
         // insert code for mouse click here
         
         // Masquer un élément 
         sym.$("play_boutton2").hide();
         
         sym.$("ra9ed")[0].currentTime = 0;
         if (sym.$("ra9ed")[0].paused) {
         	sym.$("ra9ed")[0].play();
         
         }
         
         sym.$("ra9ed")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("ra9ed")[0].currentTime >=sym.$("ra9ed")[0].duration){
           sym.$("ra9ed")[0].pause();
           sym.$("play_boutton2").show();
           sym.$("ra9ed")[0].currentTime = 0.5;
           }
         
         
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