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
           solution = new Array(1,0,0);
           reponse = new Array(0,0,0);
           
           // Hide an element 
         sym.$("vid").hide();
         sym.$("play1").hide();
         sym.$("pause").hide();
             sym.$("Attention").hide();
           // Hide an element 
           sym.$("message").hide();
             sym.$("Group").hide();
           bonne=0;
         for ( j = 0; j<3; j++) {
         
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
         
         for ( j = 0; j<3; j++) {
         	if (sym.$('E_'+j).data('clicked')){
         	nb=nb+1;
         	}
         };
         
         
         if  (nb==0) {
         sym.$("Attention").show();
         }
         else {
         for ( j = 0; j<3; j++) {
         sym.$('E_'+j).off("click");}
         sym.$("Text322").click (false);
         sym.$("GREY").click (false);
         };
         x=0;
         if  (nb!=0) {
         for ( j = 0; j<3; j++) {
         if((reponse[j]==solution[j])) {
         bonne=1;
          sym.$('CARRG'+j).delay(x).fadeOut(500);
           sym.$('CARRV'+j).delay(x).fadeIn(500)  ;
         }
         else if (reponse[j]==1){
          sym.$('CARRG'+j).delay(x).fadeOut(500);
         sym.$('CARRR'+j).delay(x).fadeIn(500)  ;}
         else if (solution[j]==1){
         sym.$('CARRG'+j).delay(x).fadeOut(500);
         sym.$('CARRR'+j).delay(x).fadeIn(500)  ;
         }
         
         x=x+200;
         }
         
         sym.$("question").delay(1200).fadeOut(500);
         sym.$("Explication").delay(1500).fadeIn(500);
         
         if (bonne==1) {
         sym.$("message").css("color", "green");
         sym.$("message").html("Bravo ! bonne reponse :)");
         }
         else
         {sym.$("message").css("color", "red");
         sym.$("message").html("Mauvaise reponse :( ");
         
         }
         
         
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text322}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text322}", "mouseover", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text322}", "mouseleave", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "mouseover", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY}", "mouseleave", function(sym, e) {
         

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
         sym.$("croix0").show();
         
         sym.$('E_0').data('clicked',true);
         reponse[0]=1;reponse[1]=0;reponse[2]=0;
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
         sym.$("croix1").show();
         
         sym.$('E_1').data('clicked',true);
         reponse[1]=1;reponse[0]=0;reponse[2]=0;
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
         sym.$("croix2").show();
         
         sym.$('E_2').data('clicked',true);
         reponse[2]=1;reponse[1]=0;reponse[0]=0;
         }
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "click", function(sym, e) {
         
         sym.$("demo").unbind("click");
         sym.$("Group").fadeIn(900);
         sym.$("Explication").hide();
         sym.$("Text3").click (false);
         sym.$("GREYCopy").click (false);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGECopy").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

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

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5000, function(sym, e) {
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