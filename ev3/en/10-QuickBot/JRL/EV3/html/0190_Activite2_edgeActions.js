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
         // Hide an element 
         sym.$("A2_exercice1").hide();
         sym.$("ZoomOut").hide();
         
         // Hide an element 
         sym.$("Sol_ex").hide();
         
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         // Masquer un élément 
         
         // Masquer un élément 
         sym.$("play1").hide();
         sym.$("pause").hide();
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

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${ZoomIn}", "click", function(sym, e) {
         // insert code for mouse click here
         sym.$("intro").fadeOut(500);
         sym.$("a7_QUICK_cable").animate({
             top: '101px',
             left: '111px',
             height: '767px',
             width: '1383px'
           },800);
         sym.$("ZoomIn").hide();
         sym.$("ZoomOut").delay(500).fadeIn(800);
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${ZoomOut}", "click", function(sym, e) {
         // insert code to be run when the mouse is moved off the object
         sym.$("intro").fadeOut(500);
         sym.$("a7_QUICK_cable").animate({
             top: '500px',
             left: '489px',
             height: '400px',
             width: '700px'
           },800);
         sym.$("ZoomOut").hide();
         sym.$("ZoomIn").delay(500).fadeIn(800);
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${indices}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         if (hint==1) 
         {
         
         
         sym.play(1000);
         hint=-1;
         
         }
         else 
         {
         
         hint=1;
         sym.playReverse(2000);
         };

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
   
   //Edge symbol: 'TITRE_2_1'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         // insérer le code ici
      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5000, function(sym, e) {
         // insérer le code ic
         sym.stop();

      });
      //Edge binding end

   })("TITRE_2_1");
   //Edge symbol end:'TITRE_2_1'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");