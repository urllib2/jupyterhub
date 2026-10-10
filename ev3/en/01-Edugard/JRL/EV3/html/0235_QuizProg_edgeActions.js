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
      
      
      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         // Masquer un élément 
         sym.$("valider").hide();
         sym.$("star_bot1").hide();
         sym.$("star_bot2").hide();
         sym.$("star_bot3").hide();
         sym.$("solution").hide();
         
         sym.$("rep2_no").hide();
         sym.$("rep1_no").hide();
         sym.$("rep3_no").hide();
         
         sym.$("rep2_yes").hide();
         sym.$("rep1_yes").hide();
         sym.$("rep3_yes").hide();
         sym.$("resultat").hide();
         // Masquer un élément 
         i=0;
         res=0;
          nb=0;
          bonne=0;
          reponse = new Array(0,0,0);
          var result = {};
         
         x=0;
         sym.$("Yes_1").draggable({
         axis: "x",
         
           start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 25, left: 25 } ,
           revert: 'invalid'	,
         
         
           });
         
           y=0;
         sym.$("Yes_22").draggable({
         axis: "x",
         
           start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 25, left: 25 } ,
           revert: 'invalid'	,
         
         
           });
         
         
         
           z=0;
          sym.$("Yes_33").draggable({
          axis: "x",
         
           start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 25, left: 25 } ,
           revert: 'invalid'	,
         
         
           });
         
           sym.$("No").draggable({
           axis: "x",
           start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 25, left: 25 } ,
           revert: 'invalid'	,
         
         
           });
         
         
           sym.$("No_2").draggable({
           axis: "x",
           start:function(e){
                     result.drag = e.target.id.split("_")[2];
                 },
           cursor: "move", cursorAt: { top: 25, left: 25 } ,
           revert: 'invalid'	,
         
         
           });
         
           sym.$("No_3").draggable({
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
         
          sym.$("rep_2").droppable({
         
         
                drop: handleDropEvent
                   });
         
          sym.$("rep_3").droppable({
         
         
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
                        {reponse[result.drag-1]=1;
         
                        bonne=bonne+1;
         
                                  }
         
         if (nb ==3){
          sym.$("valider").show();
         
          }
         
         
         
         };
         
         
         
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         q=3;
         sym.$("valider").unbind("click");
         sym.$("Message").hide();
         // Afficher un élément 
         sym.$("resultat").show();
         sym.$("nombre").html(bonne+"/"+q);
         sym.$("reset").hide();
         
         
         sym.$("Text3Copy2").click (false);
         sym.$("GREYCopy").click(false);
         sym.$("valider").click (false);
         sym.$("ORANGECopy").click (false);
         
         sym.$("reset").hide();
         
          if(sym.$("Yes_1").draggable("option","disabled") == true){
         sym.$("No").hide();
         }
         else{
         sym.$("Yes_1").hide();
         
         }
         
         if(sym.$("Yes_22").draggable("option","disabled") == true){
         sym.$("No_2").hide();
         }
         else{
         sym.$("Yes_22").hide();
         
         }
         if(sym.$("Yes_33").draggable("option","disabled") == true){
         sym.$("No_3").hide();
         }
         else{
         sym.$("Yes_33").hide();
         
         }
         sym.$("Yes").draggable( 'disable' );
         sym.$("No_1").draggable( 'disable' );
         sym.$("No_2").draggable( 'disable' );
         sym.$("Yes_22").draggable( 'disable' );
         sym.$("No_1Copy2").draggable( 'disable' );
         sym.$("Yes_3").draggable( 'disable' );
         
         
         
         if (reponse[0]==0){
          sym.$("rep_1").delay(100).fadeOut(500);
          sym.$("rep1_no").delay(100).fadeIn(500);
         }
         else
         {
         sym.$("rep_1").delay(100).fadeOut(500);
          sym.$("rep1_yes").delay(100).fadeIn(500);
         
         };
         if (reponse[1]==0){
          sym.$("rep_2").delay(300).fadeOut(500);
          sym.$("rep2_no").delay(300).fadeIn(500);
         }
         else
         {
         sym.$("rep_2").delay(300).fadeOut(500);
          sym.$("rep2_yes").delay(300).fadeIn(500);
         };
         
         if (reponse[2]==0){
          sym.$("rep_3").delay(500).fadeOut(500);
          sym.$("rep3_no").delay(500).fadeIn(500);
         }
         else
         {
         sym.$("rep_3").delay(500).fadeOut(500);
          sym.$("rep3_yes").delay(500).fadeIn(500);
         }
         
         
         
         
         
         
         
         if (bonne==3) {
         parent.Reveal.Coding[0]=3;
         
         sym.$("star_bot1").fadeIn(300);
         sym.$("star_bot2").fadeIn(400);
         sym.$("star_bot3").fadeIn(500);
         
         }
         else 
         
         if (bonne==2)
         {
         parent.Reveal.Coding[0]=2;
         sym.$("star_bot1").fadeIn(300);
         sym.$("star_bot2").fadeIn(400);
         
         }
         
         else 
         
         if (bonne==1)
         {
         parent.Reveal.Coding[0]=1;
         sym.$("star_bot1").fadeIn(300);
         };
         
         parent.Reveal.TotalCo();
         parent.Reveal.updateStarsNumber();
         
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${reset}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("rep_1").droppable( 'enable' );
         sym.$("rep_2").droppable( 'enable' );
         sym.$("rep_3").droppable( 'enable' );
         
         sym.$("Yes_1").draggable( 'enable' );
         sym.$("No").draggable( 'enable' );
         sym.$("No_2").draggable( 'enable' );
         sym.$("Yes_22").draggable( 'enable' );
         sym.$("No_3").draggable( 'enable' );
         sym.$("Yes_33").draggable( 'enable' );
          sym.$("valider").hide();
         
         nb=0;
         bonne=0;
          reponse = new Array(0,0,0);
         			 i=0;
         
         res=res+1;
         essai=3-res;
         sym.$("Message").html("You still have "+essai+" trials");
         	 if (res==3) {
         	 sym.$("reset").hide();
         	 }
         
         
         
         
         
         
             sym.$("No").animate({
                 left: "1215px",
                 top: "413px",
             });
          sym.$("Yes_1").animate({
                 left: "1281px",
                 top: "413px",
         
             });
         
         
         sym.$("No_2").animate({
                 left: "1214px",
                 top: "564px",
         
             });
         sym.$("Yes_22").animate({
                 left: "1280px",
                 top: "564px",
         
             });
         
         
         
          sym.$("No_3").animate({
                 left: "1215px",
                 top: "723px",
         
             });
         
             sym.$("Yes_33").animate({
                 left: "1281px",
                 top: "723px",
         
             });
         
         
         

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