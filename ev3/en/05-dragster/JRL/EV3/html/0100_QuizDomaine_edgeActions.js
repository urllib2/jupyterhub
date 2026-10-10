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
          
          // Masquer un élément 
          sym.$("valider").hide();
          sym.$("Text4").hide();
          
           // Masquer un élément 
         sym.$("star_bot1").hide();
         sym.$("star_bot2").hide();
         sym.$("star_bot3").hide();
         sym.$("Txt").hide();
         
         
          reponse = new Array(0,0,0,0);
         for ( j = 0; j<4; j++) {
         sym.$("CARR_ROUGE"+j).hide();
         sym.$("CARR_VERT"+j).hide();
         }
         
         
          var result = {};
          bonne=0;
          rep=0;
          res=0; //pour le bouton reset
         
         var fruit = ['drop_1','drop_2','drop_3', 'drop_4','drop_5' ,'drop_6'];
         // make an array of the droppable element names
         var baskets = ['basket_1','basket_2','basket_3', 'basket_4'];
         // have only the first element be draggable
         
         
         // set up the droppable to accept each draggables
         for (i=0;i<fruit.length;i++){
         sym.$(fruit[i]).draggable({
         start:function(e){
                     result.drag = e.target.id.split("_")[2];
         
                 },
         
         
         cursor: "move", cursorAt: { top: 40, left: 40 } ,
         revert: 'invalid' , 
         
         });
         sym.$(baskets[i]).droppable({
         // accept:'.C'+i,   // use class for accept
         drop: handleDropEvent
         });
         
         }
         
         
         
         
         function handleDropEvent(event, ui){
         
             result.drop = event.target.id.split("_")[2];
         rep=rep+1;
             if (rep==4) {
                     sym.$("valider").show();
              }  
             if(result.drag == result.drop)
                        {
                         reponse[result.drag-1]=1;
         
                        bonne=bonne+1;
                         }
         
         
             ui.draggable.position( { of: $(this), my: 'center', at: 'middle' } );
         
         }
         
         sym.$('resetBtn').click(function(){
         window.location.reload()
         });
         
         

      });
      //Edge binding end

      

    
    
      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("Text2").hide();
         q=4;
         sym.$("Txt").show();
         sym.$("nombre").html(bonne+" / "+q);
         sym.$("reset").hide();
         
         x=0;
         for ( j = 0; j<4; j++) {
         if (reponse[j]==0) {
         sym.$("CARR_GRIS"+j).hide();
         sym.$("CARR_ROUGE"+j).delay(x).fadeIn(500);
         }
         else
         {
         sym.$("CARR_GRIS"+j).hide();
         sym.$("CARR_VERT"+j).delay(x).fadeIn(500);
         }
         x=x+300;
         }
         
         // Afficher un élément 
         
         
         if (bonne==4) {
         
         sym.$("star_bot1").show();
         sym.$("star_bot2").show();
         sym.$("star_bot3").show();
         parent.Reveal.Building[0]=3;
         }
         
         else 
         if (bonne==2)
         {
         parent.Reveal.Building[0]=2;
         sym.$("star_bot1").show();
         sym.$("star_bot2").show();
         }
         
         else 
         
         if (bonne==1)
         {
         parent.Reveal.Building[0]=1;
         sym.$("star_bot1").show();
         
         };
         
         parent.Reveal.TotalBui();
         parent.Reveal.updateStarsNumber();
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

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

      Symbol.bindElementAction(compId, symbolName, "${reset}", "click", function(sym, e) {
          reponse = new Array(0,0,0,0,0);
          
         // insérer le code du clic de souris ici
         sym.$("basket_1").droppable( 'enable' );
         sym.$("basket_2").droppable( 'enable' );
         sym.$("basket_3").droppable( 'enable' );
         sym.$("basket_4").droppable( 'enable' );
         sym.$("basket_5").droppable( 'enable' );
         sym.$("basket_6").droppable( 'enable' );
          sym.$("").show()
          sym.$("valider").hide();
         
         bonne=0;
         
         
         rep=0;
         
         res=res+1;
         essai=3-res;
         sym.$("Text2").html("You still have "+essai+" trials");
         	 if (res==3) {
         	 sym.$("reset").hide();
         	 }
             sym.$("drop_6").animate({
                 left: "126px",
                 top: "359px"
             });
                 sym.$("drop_5").animate({
                 left: "991px",
                 top: "344px"
             });
         
             sym.$("drop_2").animate({
                 left: "798px",
                 top: "357px"
             });
          sym.$("drop_1").animate({
                 left: "556px",
                 top: "384px"
             });
         
         
         sym.$("drop_4").animate({
                 left: "329px",
                 top: "384px"
             });
         
             sym.$("drop_3").animate({
                 left: "1221px",
                 top: "390px"
             });
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${reset}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGECopy").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${reset}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy2}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy2}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy2").show();
         sym.$("ORANGECopy2").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy2}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy2").hide();
         sym.$("ORANGECopy2").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy2").hide();
         sym.$("ORANGECopy2").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy}", "mouseleave", function(sym, e) {
         

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