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
           reponse = new Array(0,0,0,0,0);
         i=0;
         sym.$("resultat").hide();
         sym.$("star_bot1").hide();
         sym.$("valider").hide();
         sym.$("solution").hide();
         sym.$("sol").hide();
         
         for ( j = 0; j<5; j++) {
         sym.$("CARR_ROUGE"+j).hide();
         sym.$("CARR_VERT"+j).hide();
         }
         // Masquer un élément 
         sym.$("stop").hide();
         
         
           // Masquer un élément 
         
         
          nb=0;
          bonne=0;
          res=0;
          var result = {};
         
          sym.$("tournergauche2_2").draggable({
          	start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 40, left: 40 } ,
           	revert: 'invalid'	,
            });
         
           sym.$("Avancer_1").draggable({
          	start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 40, left: 40 } ,
           	revert: 'invalid'	,
            }); 
         
         
         
         sym.$("Avancer3_1").draggable({
         start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 40, left: 40 } ,
           	revert: 'invalid'	,
         
           });
         
           sym.$("tournerdroite_3").draggable({
         start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 40, left: 40 } ,
           	revert: 'invalid'	,
         
           });
         
         
         
         sym.$("Avancer2_1").draggable({
         start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 40, left: 40 } ,
           revert: 'invalid'	,
         
         
           });
         
           //avancer droppoa
         
         
          sym.$("E_1").droppable({
         
         
                drop: handleDropEvent
                   });
           sym.$("E_3").droppable({
         
         
                drop: handleDropEvent
                   });
                    sym.$("E2_1").droppable({
         
         
                drop: handleDropEvent
                   });
            sym.$("E2_2").droppable({
         
         
                drop: handleDropEvent
                   });
             sym.$("E3_1").droppable({
         
         
                drop: handleDropEvent
                   });
         
         function handleDropEvent(event, ui){		
         
             ui.draggable.position( { of: $(this), my: 'center', at: 'middle' } );
         $(this).droppable( 'disable' );
         nb=nb+1;
         if (nb==5) {
         
            sym.$("valider").show();
            }
         
            var $this = $(this);
            result.drop = event.target.id.split("_")[2];
         if(result.drag == result.drop)
                        {reponse[i]=1;
                        bonne=bonne+1;
                        i=i+1;
                        }
         else
           {
         i=i+1;
           }
         
         
         }
         
         
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${reset}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("E_1").droppable( 'enable' );
         
         sym.$("E2_2").droppable( 'enable' );
         sym.$("E2_1").droppable( 'enable' );
         sym.$("E3_1").droppable( 'enable' );
         sym.$("E_3").droppable( 'enable' );
         
         
         
           sym.$("valider").hide();
         nb=0;
         bonne=0;
         reponse = new Array(0,0,0,0,0);
         i=0;
         
         res=res+1;
         essai=3-res;
         sym.$('Text').html("You still have "+essai+" trials");
         	 if (res==3) {
         	 sym.$("reset").hide();
         	 }
         
         
         
         
         
         
             sym.$("tournergauche2_2").animate({
                 left: "301px",
                 top: "687px",
         
             });
          sym.$("Avancer_1").animate({
                left: "401px",
                 top: "687px",
             });
         
         
         sym.$("Avancer3_1").animate({
               left: "501px",
                 top: "687px",
             });
         sym.$("tournerdroite_3").animate({
                left: "601px",
                 top: "687px",
             });
         
         
         
          sym.$("Avancer2_1").animate({
                 left: "701px",
                 top: "687px",
             });

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

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy2}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy2}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy2").hide();
         sym.$("ORANGECopy2").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy2}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy2").show();
         sym.$("ORANGECopy2").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         q=5;
         sym.$("Text").hide();
         sym.$("resultat").show();
         sym.$("nombre").html(bonne+"/"+q);
         
         
         sym.$("Text3Copy3").click (false);
         sym.$("GREYCopy3").click(false);
         sym.$("valider").click (false);
         sym.$("ORANGECopy3").click (false);
         
         
         
         sym.$("reset").hide();
         x=0;
         for ( j = 0; j<5; j++) {
         if (reponse[j]==0) {
         sym.$("CARR_ROUGE"+j).delay(x).fadeIn(500)  ; sym.$('CARR_GRIS'+j).delay(x).fadeOut(100);
         }
         else
         {
         sym.$("CARR_VERT"+j).delay(x).fadeIn(500);sym.$('CARR_GRIS'+j).delay(x).fadeOut(100);
         
         }
         x=x+300;
         }
         
         // Afficher un élément 
         
         
         
         
         if (bonne==5) {
         setTimeout(function(){ sym.getSymbol("Symbol_1").play(2000); }, 2000);
         clearTimeout();
         
         
         parent.Reveal.Coding[0]=1;
         sym.$("star_bot1").show();
         }
         else 
         {
         sym.$("valider").hide();  
         sym.$("solution").show();
         }
         parent.Reveal.TotalCo();
         parent.Reveal.updateStarsNumber();
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy3}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy3}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy3").hide();
         sym.$("ORANGECopy3").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy3}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy3").show();
         sym.$("ORANGECopy3").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy3}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy3}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy3").hide();
         sym.$("ORANGECopy3").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREYCopy3}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy3").show();
         sym.$("ORANGECopy3").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${ORANGECopy3}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${solution}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy4").hide();
         sym.$("ORANGECopy4").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${solution}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy4").show();
         sym.$("ORANGECopy4").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${solution}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("sol").show();
         
         setTimeout(function(){ sym.getSymbol("Symbol_1").play(2000); }, 4000);
         clearTimeout();
         
         sym.$("solution").hide();
         
         sym.getSymbol("sol").play();
         sym.$("E2_2").hide();
         sym.$("E3_1").hide();
         sym.$("E2_1").hide();
         sym.$("E_3").hide();
         sym.$("E_1").hide();
         sym.$("Avancer3_1").hide();
         sym.$("tournergauche2_2").hide();
         sym.$("Avancer2_1").hide();
         sym.$("tournerdroite_3").hide();
         sym.$("Avancer_1").hide();

      });
      //Edge binding end

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'sol'
   (function(symbolName) {   
   
   })("sol");
   //Edge symbol end:'sol'

   //=========================================================
   
   //Edge symbol: 'Symbol_1'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 895, function(sym, e) {
         // insérer le code ici
         sym.play(0);

      });
      //Edge binding end

   })("Symbol_1");
   //Edge symbol end:'Symbol_1'

   //=========================================================
   
   //Edge symbol: 'TITRE_1_1'
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

   })("TITRE_1_1");
   //Edge symbol end:'TITRE_1_1'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");