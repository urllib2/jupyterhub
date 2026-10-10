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
      
      
      

      

    
    
      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 3000, function(sym, e) {
         // insérer le code ici
         // insérer le code ici// insérer le code ici
         // Afficher un élément 
         sym.$("Compte_rebours").show();
         
         // Afficher un élément 
         sym.$("Reflexion").show();
         
         
         var x = 59;
         sym.$("seconde").html(x).css("color","#5abf6b");
         var id = setInterval(frame, 1000);
         function frame() {
          x--;
          if (x<10) {
          sym.$("seconde").html("0"+x).css("color","#5abf6b");
          }
          else
          {
           sym.$("seconde").html(x).css("color","#5abf6b");
         
         
          }
         if (x==0){
           clearInterval(id);
           sym.$("Reflexion").hide();
            sym.$("telechargerProg").show();
         } 
         }
         
         setTimeout(function(){ sym.$("solu").show(); sym.$("telechargerProg").hide(); }, 59000);
         clearTimeout();
         // Afficher un élément 
         

      });
      //Edge binding end

      

      Symbol.bindElementAction(compId, symbolName, "${play_boutton}", "click", function(sym, e) {
         
         // Masquer un élément 
         sym.$("play_boutton").fadeOut(500);;
         
         sym.$("Oz_test")[0].currentTime = 0;
         if (sym.$("Oz_test")[0].paused) {
         	sym.$("Oz_test")[0].play();
         
         }
         
         sym.$("Oz_test")[0].ontimeupdate = function() {videoPause()};
         
         function videoPause() {
           if (sym.$("Oz_test")[0].currentTime >=sym.$("Oz_test")[0].duration){
           sym.$("Oz_test")[0].pause();
           sym.$("play_boutton").fadeIn(500);
           sym.$("Oz_test")[0].currentTime = 0.5;
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