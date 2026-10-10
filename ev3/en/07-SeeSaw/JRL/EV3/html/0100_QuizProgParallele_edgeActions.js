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
         x=1;
         a=0;
         b=0
         c=0
         d=0
         solution = new Array(0,0,1,1);
           reponse = new Array(0,0,0,0);
           sym.$("equilibre2").hide();
           sym.$("equilibre").hide();
           sym.$("equilibre3").hide();
           sym.$("equilibreCopy3").hide();
             sym.$("equilibre2").delay(200).fadeIn(800);
                 sym.$("equilibre").delay(400).fadeIn(800);
                         sym.$("equilibre3").delay(600).fadeIn(800);
           sym.$("equilibreCopy3").delay(800).fadeIn(800);
         
           // Hide an element 
           sym.$("nombre").hide();
             sym.$("Attention").hide();
           // Hide an element 
           sym.$("message").hide();
         
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
         
         parent.Reveal.Coding[0]=1;
         sym.$("star_bot_op").hide();
         sym.$("star_bot1").show();
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

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${equilibre2}", "click", function(sym, e) {
         if (x==1){
         x=-1
         a=1
         sym.$("equilibre2").animate({
             top: '319px',
             left: '122px',
             height: '316px',
             width: '414px'
           },600);
         sym.$("equilibre3").fadeTo(300,0.1);
           sym.$("equilibreCopy3").fadeTo(300,0.1);
           sym.$("equilibre").fadeTo(300,0.1);
         }
         else if ((b==0)&&(c==0)&&(d==0))
         {
         a=0
         x=1
         sym.$("equilibre2").animate({
             top: '387px',
             left: '122px',
             height: '248px',
             width: '325px'
           },600);
         
         
           sym.$("equilibre3").fadeTo(300,1);
           sym.$("equilibreCopy3").fadeTo(300,1);
           sym.$("equilibre").fadeTo(300,1);
         
         
         
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${equilibre3}", "click", function(sym, e) {
         if (x==1){
         x=-1
         b=1
         sym.$("equilibre3").animate({
             top: '297px',
             left: '717px',
             height: '335px',
             width: '469px'
           },600);
         
         
         sym.$("equilibre2").fadeTo(300,0.1);
           sym.$("equilibreCopy3").fadeTo(300,0.1);
           sym.$("equilibre").fadeTo(300,0.1);
         
         
         
         }
         else if ((a==0)&&(c==0)&&(d==0))
         {
         x=1
         b=0
         sym.$("equilibre3").animate({
             top: '389px',
             left: '828px',
             height: '243px',
             width: '340px'
           },600);
         
         
         sym.$("equilibre2").fadeTo(300,1);
           sym.$("equilibreCopy3").fadeTo(300,1);
           sym.$("equilibre").fadeTo(300,1);
         
         
         
         
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${equilibreCopy3}", "click", function(sym, e) {
         if (x==1){
         x=-1
         c=1
         sym.$("equilibreCopy3").animate({
             top: '307px',
             left: '1097px',
             height: '331px',
             width: '390px'
           },600);
         sym.$("equilibre3").fadeTo(300,0.1)
           sym.$("equilibre2").fadeTo(300,0.1);
           sym.$("equilibre").fadeTo(300,0.1);
         
         
         
         
         
         }
         else  if ((a==0)&&(b==0)&&(d==0))
         {
         x=1
         c=0
         sym.$("equilibreCopy3").animate({
             top: '387px',
             left: '1191px',
             height: '251px',
             width: '296px'
           },600);
         
         sym.$("equilibre3").fadeTo(300,1);
           sym.$("equilibre2").fadeTo(300,1);
           sym.$("equilibre").fadeTo(300,1);
         
         
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${equilibre}", "click", function(sym, e) {
         if (x==1){
         x=-1
         d=1
         sym.$("equilibre").animate({ 
             top: '303px',
             left: '464px',
             height: '331px',
             width: '390px',
         
           },600);
         sym.$("equilibre2").fadeTo(300,0.1);
           sym.$("equilibreCopy3").fadeTo(300,0.1);
           sym.$("equilibre3").fadeTo(300,0.1);
         
         
         
         }
         else if ((b==0)&&(c==0)&&(a==0))
         {
         x=1
         d=0
         sym.$("equilibre").animate({
             top: '387px',
             left: '495px',
             height: '248px',
             width: '291px'
           },600);
         sym.$("equilibre2").fadeTo(300,1);
           sym.$("equilibreCopy3").fadeTo(300,1);
           sym.$("equilibre3").fadeTo(300,1);
         
         
         
         
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