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
           solution = new Array(1,0,0,1);
           reponse = new Array(0,0,0,0);
           sym.$("Choix1").hide();
           sym.$("Choix6").hide();
           sym.$("Choix4").hide();
           sym.$("Choix42").hide();
           sym.$("Choix6").delay(200).fadeIn(800);
           sym.$("Choix4").delay(400).fadeIn(800);
           sym.$("Choix42").delay(600).fadeIn(800);
           sym.$("Choix1").delay(800).fadeIn(800);
           
           // Hide an element 
           sym.$("nombre").hide();
           sym.$("Attention").hide();
           // Hide an element 
           sym.$("message").hide();
           sym.$("star_bot1").hide();
           bonne=0;
         for ( j = 0; j<4; j++) {
         
         sym.$("CARRR"+j).hide();
         sym.$("CARRV"+j).hide();
         sym.$("croix"+j).hide();
         sym.$("CARRG"+j).data('clicked', false);
         
         }
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         
         nb=0;
         q=4;
         for ( j = 0; j<4; j++) {
         	if (sym.$('CARRG'+j).data('clicked')){
         	nb=nb+1;
         	}
         };
         if  (nb==0) {
         sym.$("Attention").show();
         }
         else {
         sym.$("bouton").unbind("click");
         }
         
         x=0;
         if (nb!=0){
         sym.$("Attention").hide();
         
         for ( j = 0; j<4; j++) {
         if(reponse[j]==solution[j]) {
         bonne=bonne+1;
         
           sym.$('CARRV'+j).delay(x).fadeIn(500)  ; sym.$('CARRG'+j).delay(x).fadeOut(100);
         }
         else
         {
         
         sym.$('CARRR'+j).delay(x).fadeIn(500) ;sym.$('CARRG'+j).delay(x).fadeOut(100);
         }
         x=x+300;
         }
         
         // Show an element 
         sym.$("message").show();
         
         // Show an element 
         sym.$("nombre").show();
         
         sym.$("nombre").html(bonne+"/"+q);
         if (bonne==4) {
         sym.$("star_bot1").show();
         parent.Reveal.Knowledge[1]=1;;
         
         }
         
         parent.Reveal.updateStarsNumber();
         parent.Reveal.TotalKnow();
         
         }

      });
      //Edge binding end

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${croix0}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         
         if(sym.$('CARRG0').data('clicked')) { 
         sym.$("croix0").hide();
         
         sym.$('CARRG0').data('clicked',false);
         reponse[0]=0;
         }
         else
         {
         sym.$("croix0").show();
         
         sym.$('CARRG0').data('clicked',true);
         reponse[0]=1;
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${CARRG0}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         sym.$("Attention").hide();
         if(sym.$('CARRG0').data('clicked')) { 
         sym.$("croix0").hide();
         
         sym.$('CARRG0').data('clicked',false);
         reponse[0]=0;
         }
         else
         {
         sym.$("croix0").show();
         
         sym.$('CARRG0').data('clicked',true);
         reponse[0]=1;
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${croix1}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         
         if(sym.$('CARRG1').data('clicked')) { 
         sym.$("croix1").hide();
         
         sym.$('CARRG1').data('clicked',false);
         reponse[1]=0;
         }
         else
         {
         sym.$("croix1").show();
         
         sym.$('CARRG1').data('clicked',true);
         reponse[1]=1;
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${CARRG1}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         sym.$("Attention").hide();
         if(sym.$('CARRG1').data('clicked')) { 
         sym.$("croix1").hide();
         
         sym.$('CARRG1').data('clicked',false);
         reponse[1]=0;
         }
         else
         {
         sym.$("croix1").show();
         
         sym.$('CARRG1').data('clicked',true);
         reponse[1]=1;
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${croix2}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         
         if(sym.$('CARRG2').data('clicked')) { 
         sym.$("croix2").hide();
         
         sym.$('CARRG2').data('clicked',false);
         reponse[2]=0;
         }
         else
         {
         sym.$("croix2").show();
         
         sym.$('CARRG2').data('clicked',true);
         reponse[2]=1;
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${CARRG2}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         sym.$("Attention").hide();
         if(sym.$('CARRG2').data('clicked')) { 
         sym.$("croix2").hide();
         
         sym.$('CARRG2').data('clicked',false);
         reponse[2]=0;
         }
         else
         {
         sym.$("croix2").show();
         
         sym.$('CARRG2').data('clicked',true);
         reponse[2]=1;
         }

      });
      //Edge binding end

      

      

      Symbol.bindElementAction(compId, symbolName, "${croix3}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         
         if(sym.$('CARRG3').data('clicked')) { 
         sym.$("croix3").hide();
         
         sym.$('CARRG3').data('clicked',false);
         reponse[3]=0;
         }
         else
         {
         sym.$("croix3").show();
         
         sym.$('CARRG3').data('clicked',true);
         reponse[3]=1;
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${CARRG3}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         sym.$("Attention").hide();
         if(sym.$('CARRG3').data('clicked')) { 
         sym.$("croix3").hide();
         
         sym.$('CARRG3').data('clicked',false);
         reponse[3]=0;
         }
         else
         {
         sym.$("croix3").show();
         
         sym.$('CARRG3').data('clicked',true);
         reponse[3]=1;
         }

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "mouseleave", function(sym, e) {
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