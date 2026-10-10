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
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         // Masquer un élément 
         
         // Masquer un élément 
         sym.$("play1").hide();
         sym.$("pause").hide();
         sym.$("Ex1_son").hide();
         play=1;
         hint=1;
         demo=1;
         sym.$("full_screen").hide();
         sym.getSymbol("video").stop(0);
         
         
         // Masquer un élément 
         sym.$("Solution_Ex1").hide();
         
         
         
         // Masquer un élément 
         
         
         // Masquer un élément 
         sym.$("reflechir").hide();
         
         // Masquer un élément 
         sym.$("Rectangle2").hide();
         sym.$("solu").hide();
         
         // Masquer un élément 
         sym.$("solution").hide();
         
         sym.$("demonstration").hide();
         // Masquer un élément 
         
         
         
         
         sym.play();
         
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

            



      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 897, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2003, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         
         // insérer le code ici
         
         // insérer le code ici// insérer le code ici
         // Afficher un élément 
         sym.$("Compte_rebours").show();
         
         // Afficher un élément 
         sym.$("reflexion").show();
         
          y=1;
         x =59;
         sym.$("secondes").html(x).css("color","#5abf6b");
         sym.$("minutes").html("0"+y+":").css("color","#5abf6b");
         
         
         var id = setInterval(frame, 1000);
         function frame() {
         if ((x<=0)&&(y<=0)){
           clearInterval(id);
         
           sym.$("Reflexion").hide();
           sym.$("solu").show(); sym.$("demonstration").hide();
           sym.$("demo").hide();
           sym.$("Hint").hide();
         
         } 
         
          if (y==0){
          x--;
          if (x<10) {
          sym.$("secondes").html("0"+x).css("color","#5abf6b");
          sym.$("minutes").html("0"+y+":").css("color","#5abf6b");
          }
          else
          {
           sym.$("secondes").html(x).css("color","#5abf6b");
         sym.$("minutes").html("0"+y+":").css("color","#5abf6b");
         
          }
          }
          else
          {
          if (x<=10){
          if (x==0){
          x=59;
          y--;
          sym.$("secondes").html(x).css("color","#5abf6b");
          sym.$("minutes").html("0"+y+":").css("color","#5abf6b");
         
          }
          else{
          x--;
          sym.$("secondes").html("0"+x).css("color","#5abf6b");
          sym.$("minutes").html("0"+y+":").css("color","#5abf6b");
              }
          }
         else {
         x--;
         sym.$("minutes").html("0"+y+":").css("color","#5abf6b");
         sym.$("secondes").html(x).css("color","#5abf6b");
         
         
         }  
         
          }
         
         
         
         
          }
         
         
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         // insérer le code ici
         
         // insérer le code ici// insérer le code ici
         // Afficher un élément 
         sym.$("Compte_rebours").show();
         
         // Afficher un élément 
         sym.$("reflexion").show();
         
         var y=0;
         var x = 10;
         sym.$("seconde").html(x).css("color","#5abf6b");
         sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         var x = 10;
         
         var id = setInterval(frame, 1000);
         function frame() {
         if ((x==0)&&(y==0)){
           clearInterval(id);
           sym.$("Reflexion").hide();
           sym.$("solu").show(); sym.$("demonstration").hide();
           sym.$("demo").hide();
           sym.$("Hint").hide();
         } 
         
          if (y==0){
          x--;
          if (x<10) {
          sym.$("seconde").html("0"+x).css("color","#5abf6b");
          sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
          }
          else
          {
           sym.$("seconde").html(x).css("color","#5abf6b");
         sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         
          }
          }
          else
          {
          if (x<=10){
          if (x==0){
          x=59;
          y--;
          sym.$("seconde").html(x).css("color","#5abf6b");
          sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         
          }
          else{
          x--;
          sym.$("seconde").html("0"+x).css("color","#5abf6b");
          sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
              }
          }
         else {
         x--;
         sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         sym.$("seconde").html(x).css("color","#5abf6b");
         
         
         }  
         
          }
         
         
         
         
          }
         
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${AS_bouclecompteur}", "mouseover", function(sym, e) {
         sym.$("intro").fadeOut(400);
         sym.$("AS_bouclecompteur").animate({
             top: '99px',
             left: '270px',
             height: '815px',
             width: '1076px'
           });
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${AS_bouclecompteur}", "mouseleave", function(sym, e) {
         sym.$("intro").fadeIn(500);
         sym.$("AS_bouclecompteur").animate({
             top: '298px',
             left: '398px',
             height: '613px',
             width: '835px'
           });
         

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