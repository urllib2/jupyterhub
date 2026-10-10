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
         
         sym.$("Group3").hide();
         sym.$("Group2").hide();
         sym.$("Group1").hide();
         sym.$("Green").hide();
         sym.$("LightGreen").hide();
         sym.$("Yellow1").hide();
         sym.$("Yellow").hide();
         sym.$("Orange").hide();
         sym.$("Red").hide();
         sym.$("Resultat").hide();
         

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

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3Copy}", "mouseleave", function(sym, e) {
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

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
           sym.$("Group3").delay(600).fadeIn(500);
           sym.$("Group1").delay(400).fadeIn(500);
           sym.$("Group2").delay(200).fadeIn(500);
           sym.$("resultat").delay(200).fadeIn(800);
         
         NbrEtoileTot=10;
         NbrEtoileConn=7;
         NbrEtoileProg=3;
         NbrEtoileMeca=5;
         
         s=parent.Reveal.updateStarsNumber();
         Khowl=parent.Reveal.TotalKnow();
         Cod=parent.Reveal.TotalCo();
         Build=5//parent.Reveal.TotalBui();
         
         Ratio=s/NbrEtoileTot;
         l=Ratio*700;
         
         sym.$("Aiguille").animate({
             top: '0px',
             left: -l+'px',
           },800);
         if((l>=0)&&(l<90))
         {
         sym.$("Red").delay(200).fadeIn(800);
         }
         else if((l>100)&&(l<220))
         {
         sym.$("Orange").delay(200).fadeIn(800);
         }
         else if((l>220)&&(l<360))
         {
         sym.$("Yellow").delay(200).fadeIn(800);
         }
         else if((l>360)&&(l<510))
         {
         sym.$("Yellow1").delay(200).fadeIn(800);
         }
         else if((l>510)&&(l<650))
         {
         sym.$("LightGreen").delay(200).fadeIn(800);
         }
         else if(l>650)
         {
         sym.$("Green").delay(200).fadeIn(800);
         }
         
         
         
         sym.$("BarKhowledgeProgress").animate({
             width: 600*(Khowl/NbrEtoileConn)+'px',
           },800);
         
         sym.$("BarBuildingProg").animate({
             width: 600*(Build/NbrEtoileMeca)+'px',
           },800);
         sym.$("BarCodingProgress").animate({
             width: 600*(Cod/NbrEtoileProg)+'px',
           },800);
         if(s<=0.3*NbrEtoileTot)
         {
         sym.$("Remarque").css("color", "Red");
         sym.$("Remarque").html("Weak")
         }
         if((s>0.3*NbrEtoileTot)&&(s<0.6*NbrEtoileTot))
         {
         sym.$("Remarque").css("color", "#ff8000");
         sym.$("Remarque").html("Average");
         }
         if((s>=0.6*NbrEtoileTot)&&(s<0.8*NbrEtoileTot))
         {
         sym.$("Remarque").css("color", "#ffcc00");
         sym.$("Remarque").html("Good")
         }
         if(s>=0.8*NbrEtoileTot)
         {
         sym.$("Remarque").css("color", "Green");
         sym.$("Remarque").html("Excellent")
         }
         sym.$("Resultat").show();
         sym.$("Nombre").html(s)
         sym.$("Total").html(NbrEtoileTot)
         sym.$("PourcentageProg").html(((Cod/NbrEtoileProg)*100).toFixed(2)+'%');
         sym.$("PourcentageMec").html(((Build/NbrEtoileMeca)*100).toFixed(2)+'%');
         sym.$("PourcentageConn").html(((Khowl/NbrEtoileConn)*100).toFixed(2)+'%');
         sym.$("bouton").unbind("click");

      });
      //Edge binding end

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'TITRE'
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

   })("TITRE");
   //Edge symbol end:'TITRE'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");