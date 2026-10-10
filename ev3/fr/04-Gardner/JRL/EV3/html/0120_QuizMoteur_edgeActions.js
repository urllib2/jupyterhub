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

      Symbol.bindElementAction(compId, symbolName, "${Rectangle2Copy3}", "click", function(sym, e) {
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
         sym.$("star_bot3").hide();
         sym.$("star_bot2").hide();
         sym.$("Rectangle2Copy").data('clicked', false);
         sym.$('Rectangle2').data('clicked',false);
         sym.$('Rectangle2Copy3').data('clicked',false);
         sym.$('Rectangle2Copy4').data('clicked',false);
         sym.$('Rectangle2Copy7').data('clicked',false);
         sym.$('Rectangle2Copy5').data('clicked',false);
         sym.$("Text2").hide();
         
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         
         
         
         if ((sym.$('Rectangle2Copy').data('clicked'))||(sym.$('Rectangle2').data('clicked'))||(sym.$('Rectangle2Copy3').data('clicked'))){
         sym.$("Text2").hide();
         sym.$("blo1").unbind("click");
         sym.$("blo2").unbind("click");
         sym.$("blo6").unbind("click");
         sym.$("Rectangle2").unbind("click");
         sym.$("Rectangle2Copy").unbind("click");
         sym.$("Rectangle2Copy3").unbind("click");
         if ((sym.$('Rectangle2Copy').data('clicked')==false)&&(sym.$('Rectangle2').data('clicked')==true)&&(sym.$('Rectangle2Copy3').data('clicked')==true)){
         sym.$("star_bot2").show();
         sym.$("star_bot3").show();
         sym.$('Rectangle2').css("background-color","green") ;
         sym.$('Rectangle2Copy3').css("background-color","green") ;
         sym.$("Text2").hide();
         sym.$("bouton").hide();
         parent.Reveal.Coding[1]=2;
         
         }
         else if ((sym.$('Rectangle2Copy').data('clicked')==true)&&((sym.$('Rectangle2').data('clicked')==true)&&(sym.$('Rectangle2Copy3').data('clicked')==true)))
         {
         sym.$('Rectangle2Copy').css("background-color","red");
         sym.$('Rectangle2').css("background-color","green");
         sym.$('Rectangle2Copy3').css("background-color","green");
         sym.$("star_bot2").show();
         parent.Reveal.Coding[1]=2;
         
         }
         else if ((sym.$('Rectangle2Copy').data('clicked')==true)||((sym.$('Rectangle2').data('clicked')==true)||(sym.$('Rectangle2Copy3').data('clicked')==true)))
         {
         if(sym.$('Rectangle2Copy').data('clicked')) { 
         sym.$('Rectangle2Copy').css("background-color","red")  ;}
         if(sym.$('Rectangle2').data('clicked')) { 
         sym.$("Rectangle2").css("background-color","green")
         sym.$("Rectangle2Copy3").css("background-color","Red")
         sym.$("star_bot2").show();
         parent.Reveal.Coding[1]=1;}
         
         
         if(sym.$('Rectangle2Copy3').data('clicked')) { 
         sym.$("Rectangle2Copy3").css("background-color","green")
         sym.$("Rectangle2").css("background-color","Red")
         sym.$("star_bot2").show();
         parent.Reveal.Coding[1]=1;}
         
         
         sym.$("Text2").hide();
         sym.$("bouton").hide();
         
         
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

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${play_boutton}", "click", function(sym, e) {
         
         // Masquer un élément 
         sym.$("play_boutton").fadeOut(500);
         sym.$("quiz_frein")[0].currentTime = 0;
         if (sym.$("quiz_frein")[0].paused) {
         setTimeout(function(){ sym.$("quiz_frein")[0].play(); }, 0);
                                     clearTimeout();
         
         }
         
         sym.$("quiz_frein")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("quiz_frein")[0].currentTime >=sym.$("quiz_frein")[0].duration){
           sym.$("quiz_frein")[0].pause();
           sym.$("quiz_frein").show();
           sym.$("quiz_frein")[0].currentTime = 0.0;
             sym.$("play_boutton").fadeIn(500);
           }
         
         
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