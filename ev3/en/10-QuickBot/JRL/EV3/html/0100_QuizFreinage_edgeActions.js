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
           solution = new Array(0,0,1,0);
           reponse = new Array(0,0,0,0);
           
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
         sym.$("E_"+j).data('clicked', false);
         }

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         nb=0;
         
         q=1;
         
         
         for ( j = 0; j<4; j++) {
         	if (sym.$('E_'+j).data('clicked')){
         	nb=nb+1;
         	}
         };
         
         if  (nb==0) {
         sym.$("Attention").show();
         }
         else {
         for ( j = 0; j<4; j++) {
         sym.$('CARRG'+j).off("click");}
         sym.$("Text3").click (false);
         sym.$("GREY").click (false);
         };
         
         if  (nb!=0) {
         for ( j = 0; j<4; j++) {
         if((reponse[j]==solution[j])&&(solution[j]==1)){
         bonne=1;
          sym.$('CARRG'+j).hide();
           sym.$('CARRV'+j).show();
         }
         else if (reponse[j]==1){
          sym.$('CARRG'+j).hide();
         sym.$('CARRR'+j).show()  ;}
         else if (solution[j]==1){
         sym.$('CARRG'+j).hide();
         sym.$('CARRV'+j).show() ;
         }
         
         }
         
         sym.$("message").show();
         
         if (bonne==1) {
         sym.$("message").css("color", "green");
         sym.$("message").html("Well done ! right answer :)");
         parent.Reveal.Coding[0]=1;
         sym.$("star_bot1").show();
         }
         else
         {sym.$("message").css("color", "red");
         sym.$("message").html("Wrong answer :( ");
         }
         
         
         parent.Reveal.TotalCo();
         parent.Reveal.updateStarsNumber();
         
         }
         
         
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGECopy").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGECopy").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Hide an element 
         sym.$("GREY").hide();
         
         // Show an element 
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Hide an element 
         sym.$("ORANGE").hide();
         
         // Show an element 
         sym.$("GREY").show();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${E_3}", "click", function(sym, e) {
              sym.$("Attention").hide();
         if(sym.$('E_3').data('clicked')) { 
         sym.$("croix3").hide();
         
         sym.$('E_3').data('clicked',false);
         reponse[3]=0;
         }
         else
         {
         sym.$("croix0").hide();
         sym.$('E_1').data('clicked',false);
         sym.$("croix2").hide();
         sym.$('E_0').data('clicked',false);
         sym.$("croix1").hide();
         sym.$('E_2').data('clicked',false);
         sym.$("croix3").show();
         
         sym.$('E_3').data('clicked',true);
         reponse[3]=1;reponse[0]=0;reponse[2]=0;reponse[1]=0;
         }
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${E_0}", "click", function(sym, e) {
              sym.$("Attention").hide();
         if(sym.$('E_0').data('clicked')) { 
         sym.$("croix0").hide();
         sym.$('E_0').data('clicked',false);
         reponse[0]=0;
         }
         else
         {
         sym.$("croix1").hide();
         sym.$('E_1').data('clicked',false);
         sym.$("croix2").hide();
         sym.$('E_2').data('clicked',false);
         sym.$("croix3").hide();
         sym.$('E_3').data('clicked',false);
         sym.$("croix0").show();
         
         sym.$('E_0').data('clicked',true);
         reponse[0]=1;reponse[1]=0;reponse[2]=0;reponse[3]=0;
         }
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${E_1}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
         // insérer le code du clic de souris ici
             sym.$("Attention").hide();
         if(sym.$('E_1').data('clicked')) { 
         sym.$("croix1").hide();
         
         sym.$('E_1').data('clicked',false);
         reponse[1]=0;
         }
         else
         {
         sym.$("croix0").hide();
         sym.$('E_0').data('clicked',false);
         sym.$("croix2").hide();
         sym.$('E_2').data('clicked',false);
         sym.$("croix3").hide();
         sym.$('E_3').data('clicked',false);
         sym.$("croix1").show();
         
         sym.$('E_1').data('clicked',true);
         reponse[1]=1;reponse[0]=0;reponse[2]=0;reponse[3]=0;
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${E_2}", "click", function(sym, e) {
              sym.$("Attention").hide();
         if(sym.$('E_2').data('clicked')) { 
         sym.$("croix2").hide();
         
         sym.$('E_2').data('clicked',false);
         reponse[2]=0;
         }
         else
         {
         sym.$("croix1").hide();
         sym.$('E_1').data('clicked',false);
         sym.$("croix0").hide();
         sym.$('E_0').data('clicked',false);
         sym.$("croix3").hide();
         sym.$('E_3').data('clicked',false);
         sym.$("croix2").show();
         
         sym.$('E_2').data('clicked',true);
         reponse[2]=1;reponse[1]=0;reponse[0]=0;reponse[3]=0;
         }
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${play_boutton}", "click", function(sym, e) {
         
         // Masquer un élément 
         sym.$("play_boutton").fadeOut(500);
         
         sym.$("quiz_frein")[0].currentTime = 0;
         if (sym.$("quiz_frein")[0].paused) {
         setTimeout(function(){ sym.$("quiz_frein")[0].play(); }, 2000);
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