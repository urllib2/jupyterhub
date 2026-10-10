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
          
          
           // Masquer un élément 
           sym.$("star_bot1").hide();
           sym.$("star_bot2").hide();
          
          
          
            sym.$("resultat").hide();
          reponse = new Array(0,0);
          
          var result = {};
          bonne=0;
          rep=0;
          res=0; //pour le bouton reset
          
         var fruit = ['drop_1','drop_2'];
         // make an array of the droppable element names
         var baskets = ['basket_1','basket_2'];
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
             $(this).droppable( 'disable' );
             result.drop = event.target.id.split("_")[2];
         rep=rep+1;
             if (rep==2) {
                     sym.$("valider").show();
              }
             if(result.drag == result.drop)
                        {
                        bonne=bonne+1;
         
                        reponse[result.drag-1]=1;
         
         
         			  }
         
         
         
             ui.draggable.position( { of: $(this), my: 'center', at: 'middle' } );
         
         }
         
         sym.$('resetBtn').click(function(){
         window.location.reload()
         });
         
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${reset}", "click", function(sym, e) {
          reponse = new Array(0,0);
          rep=0;
         // insérer le code du clic de souris ici
         sym.$("basket_1").droppable( 'enable' );
         sym.$("basket_2").droppable( 'enable' );
         sym.$("valider").hide();
         res=res+1;
         essai=3-res;
         sym.$("Text").html("Il vous reste "+essai+" essais");
         	 if (res==3) {
         	 sym.$("reset").hide();
         	 }
         
             sym.$("drop_2").animate({
                 left: "465px",
                 top: "260px"
             });
          sym.$("drop_1").animate({
                 left: "853px",
                 top: "260px"
             });
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${reset}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${reset}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANGE").show();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy2").show();
         sym.$("ORANGECopy2").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy2").hide();
         sym.$("ORANGECopy2").show();

      });
      //Edge binding end

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         
         // insérer le code du clic de souris ici
         sym.$("reset").hide();
         sym.$("Text").hide();
         if (reponse[0]==0) 
         {
         sym.$("RoundRect1").css("border-color"," Red");
         sym.$("basket_1").css("border-color"," Red");
         }
         else
         {
         sym.$("RoundRect1").css("border-color","Green")
         sym.$("basket_1").css("border-color","Green")
         }
         if (reponse[1]==0) 
         {
         sym.$("RoundRect2").css("border-color"," Red");
         sym.$("basket_2").css("border-color"," Red");
         }
         else
         {
         sym.$("RoundRect2").css("border-color","Green")
         sym.$("basket_2").css("border-color","Green")
         }
         bonne=reponse[0]+reponse[1];
         if (bonne==2)
         {
         sym.$("star_bot1").show();
         sym.$("star_bot2").show();
         parent.Reveal.Coding[0]=2;	
         }
         
         parent.Reveal.TotalCo();
         parent.Reveal.updateStarsNumber();

      });
      //Edge binding end

      

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy2}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy2}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGECopy").hide();

      });
      //Edge binding end

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${valider}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGECopy").hide();

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