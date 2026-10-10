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
         
         sym.$("question").hide();
         
         // Masquer un élément 
         sym.$("Rectangle2").hide();
         sym.$("solu").hide();
         sym.$("Explication").hide();
         // Masquer un élément 
         sym.$("solution").hide();
         
         
         
         sym.play();
         
         var result = {};
         
         nb=0;
         i=0;
         bonne=0;
         reponse=0;
         sym.$("Yes").draggable({
         axis: "x",
         
           start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 25, left: 25 } ,
           revert: 'invalid'	,
         
         
           });
         
         
           sym.$("No_1").draggable({
           axis: "x",
           start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 25, left: 25 } ,
           revert: 'invalid'	,
         
         
           });
         
         
         
           //avancer droppoa
         
         
          sym.$("rep_1").droppable({
                drop: handleDropEvent
                   });
         
         
         function handleDropEvent(event, ui){		
         
         ui.draggable.position( { of: $(this), my: 'center', at: 'middle' } );
         var $this = $(this);
         $(this).droppable( 'disable' );
         ui.draggable.draggable('option','disabled', true ); 
         nb=nb+1;
         
         
           result.drop = event.target.id.split("_")[2];
         if(result.drag == result.drop)
                        {reponse=1;
                        bonne=bonne+1;
                        }
         
         if (nb ==1){
          sym.$("valider").show();
         
          }
         
         
         
         };
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

            



      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 883, function(sym, e) {
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

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${demo}", "click", function(sym, e) {
         
         sym.$("question").fadeIn(900);
         sym.$("Explication").hide();
         sym.$("valider").hide();
         sym.$("demo").fadeOut(300);
         sym.$("Text3").click (false);
         sym.$("GREY").click (false);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         
         
         // Afficher un élément 
         sym.$("resultat").show();
         
         
         
         
         
         
         sym.$("Text3Copy").click (false);
         sym.$("GREYCopy").click(false);
         sym.$("valider").click (false);
         sym.$("ORANGECopy").click (false);
         
         
         
         
          if(sym.$("Yes").draggable("option","disabled") == true){
         sym.$("No_1").hide();
         }
         else{
         sym.$("Yes").hide();
         
         }
         
         
         
         sym.$("Yes").draggable( 'disable' );
         sym.$("No_1").draggable( 'disable' );
         
         
         
         
         if (reponse==0){
          sym.$("rep_1").fadeOut(500);
          sym.$("rep1_no").fadeIn(500);
         }
         else
         {
         sym.$("rep_1").fadeOut(500);
          sym.$("rep1_no").hide();
          sym.$("rep1_yes").fadeIn(500);
         
         };
         
         sym.$("Explication").delay(4000).fadeIn(500);
         sym.$("camarade").delay(4000).fadeOut(500);
         sym.$("valider").delay(20).fadeOut(500);
         

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
   
   //Edge symbol: 'TITRE_2'
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

   })("TITRE_2");
   //Edge symbol end:'TITRE_2'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");