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
         sym.$('Text1').html("<input type='number'  id='input1' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:55px; letter-spacing: 2px; color: black; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text2').html("<input type='number'  id='input2' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:55px; letter-spacing: 2px; color: black; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text3').html("<input type='number'  id='input3' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:55px; letter-spacing: 2px; color: black; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('Text4').html("<input type='number'  id='input4' placeholder='........'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:180px; height:55px; letter-spacing: 2px; color: black; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         sym.$('star_bot1').hide();
         sym.$('star_bot2').hide();
         sym.$('star_bot3').hide();
         sym.$("h2").hide();
         sym.$("Note").hide();
         sym.$("Message").hide();
         sym.$("Message2").hide();
         sym.$("Total").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "document", "keyup", function(sym, e) {
         // insérer le code à exécuter lorsque vous cessez d'appuyer sur une touche.
         // La variable “e.which” indique le code associé à la touche sur laquelle vous avez appuyé, par exemple 13 = entrée
         if (e.which == 13) {
         sym.$("valider").click();
         
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
             
             x=1;
             y=3;
             z=50;
             w=2;
             rep=0
          if((input1.value.length==0)||(input2.value.length==0)||(input3.value.length==0)||(input4.value.length==0))
            {
         	 sym.$('Message2').show();
         	}
         
         else 
         { 
              input1.disabled = true; 
              input2.disabled = true; 
              input3.disabled = true; 
              input4.disabled = true;
              sym.$('Message2').hide();
              sym.$('valider').unbind();
         
             if ((input1.value==x)&&(input2.value==y)&&(input3.value==z)&&(input4.value==w))
              {
         		sym.$('star_bot1').show();
         		sym.$('star_bot2').show();
         		sym.$('star_bot3').show();
         		sym.$("Message").show();
         		sym.$("Note").show();
         		sym.$("Total").show();
         		sym.$("Note").html("4");
         		parent.Reveal.Coding[1]=3;
         
              }
             else
         	{
         
         			if(input1.value==x)
         				{rep=rep+1;}
         			if(input2.value==y)
         				{rep=rep+1;}
         			if(input3.value==z)
         				{rep=rep+1;}
         			if(input4.value==w)
         				{rep=rep+1;}
         			if(rep==0)
         			{
         
         			sym.$("Message").show();
         			sym.$("Note").show();
         			sym.$("Total").show();
         			sym.$("Note").html(rep);
         			parent.Reveal.Coding[1]=0;
         			}	
         
         
         			else if((rep==1)||(rep==2))
         			{
         			sym.$('star_bot1').show();
         			sym.$("Message").show();
         			sym.$("Note").show();
         			sym.$("Total").show();
         			sym.$("Note").html(rep);
         			parent.Reveal.Coding[1]=1;
         			}
         			else 
         			{
         			sym.$('star_bot1').show();
         			sym.$('star_bot2').show();
         			sym.$("Message").show();
         			sym.$("Note").show();
         			sym.$("Total").show();
         			sym.$("Note").html(rep);
         			parent.Reveal.Coding[1]=2;
         			}			
         	}
         };
         
         parent.Reveal.TotalCo();
         parent.Reveal.updateStarsNumber();
         
         

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

      

      Symbol.bindElementAction(compId, symbolName, "${Rectangle}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("h2").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Rectangle}", "mouseenter", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris entre dans un élément
         sym.$("h2").show();

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