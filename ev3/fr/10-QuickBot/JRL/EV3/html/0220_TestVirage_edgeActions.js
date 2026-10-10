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
         
         sym.$('Text1').html("<input type='number' id='input1' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:67px; letter-spacing: 2px; color: red; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text2').html("<input type='number' id='input2' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='3' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:67px; letter-spacing: 2px; color: red; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text3').html("<input type='number' id='input3' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='5' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:67px; letter-spacing: 2px; color: red; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text4').html("<input type='number' id='input4' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='7' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:67px; letter-spacing: 2px; color: red; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         
         sym.$('Text5').html("<input type='number' id='input5' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='2' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:67px; letter-spacing: 2px; color: red; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text6').html("<input type='number' id='input6' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='4' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:67px; letter-spacing: 2px; color: red; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         
         sym.$('Text7').html("<input type='number' id='input7' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='6' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:67px; letter-spacing: 2px; color: red; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text8').html("<input type='number' id='input8' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='8' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:67px; letter-spacing: 2px; color: red; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5500, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         
         x=0;
         
         sym.$("message").show();
         sym.$('message').html('');
         
         
             if((input1.value>0)&&(input1.value<1000)&&(input2.value>0)&&(input2.value<1000)&&(input3.value<1000)&&(input3.value>0)&&(input4.value<1000)&&(input4.value>0)&&(input5.value<1000)&&(input5.value>0)&&(input6.value<1000)&&(input6.value>0)&&(input7.value<1000)&&(input2.value<1000)&&(input7.value>0)&&(input8.value<1000)&&(input8.value>0) ){
         		sym.$('message').html('Valeurs enregistrées!');
         		input1.disabled = true;
         		input2.disabled = true;
         		input3.disabled = true;
         		input4.disabled = true;
         		input5.disabled = true;
         		input6.disabled = true;
         		input7.disabled = true;
         		input8.disabled = true;
         
              sym.$("Text3").click (false);
              sym.$("GREY").click (false);
         
              parent.Reveal.Mesure[0]= parseInt(input1.value);
         	  parent.Reveal.Mesure[1]= parseInt(input2.value);
         		parent.Reveal.Mesure[2]= parseInt(input3.value);
         		parent.Reveal.Mesure[3]= parseInt(input4.value);
         		parent.Reveal.Mesure[4]= parseInt(input5.value);
         	 parent.Reveal.Mesure[5]= parseInt(input6.value);
         	 parent.Reveal.Mesure[6]= parseInt(input6.value);
         	 parent.Reveal.Mesure[7]= parseInt(input6.value);
         	}
         	else{
         	sym.$('message').html('Vérifier vos valeurs!');
         
         	}	;	
         
         
         
         setTimeout(function(){ sym.$("message").hide(); }, 1500);
         clearTimeout();
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "document", "keyup", function(sym, e) {
         // insérer le code à exécuter lorsque vous cessez d'appuyer sur une touche.
         // La variable “e.which” indique le code associé à la touche sur laquelle vous avez appuyé, par exemple 13 = entrée
         if (e.which == 13) {
         sym.$("valider").click();
         	// exécuter une action
         }

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

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");