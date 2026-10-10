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
      
      
      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${bouton}", "click", function(sym, e) {
         x=2;
         y=25;
         z=50;
         w=-50;
         b=3;
         c=2;
         correct=0;
         
         
         if((input1.value.length==0)||(input2.value.length==0)||(input3.value.length==0)||(input4.value.length==0)||(input5.value.length==0)||(input6.value.length==0))
            {
         		sym.$('Alert').show();
         	}
         
         else 
         { 
         	  input1.disabled = true; 
              input2.disabled = true; 
              input3.disabled = true; 
              input4.disabled = true;
              input5.disabled = true;
              input6.disabled = true;
         
              sym.$('Alert').hide();
              sym.$('valider').unbind();
         
             if((input1.value==x)&&(input2.value==y)&&(input3.value==z)&&(input4.value==w)&&(input5.value==b)&&(input6.value==c))
         		{
         			for (j=1;j<7;j++)
         			{
         			sym.$("Vrai"+j).show();
         			}
         		sym.$('star_bot1').show();
         		sym.$('star_bot2').show();
         		sym.$('star_bot3').show();
         		parent.Reveal.Coding[0]=3;
         		}
         	 else
         	{
         			if(input1.value==x)
         				{
         				sym.$("Vrai1").show();
         				correct=correct+0.5;
         				}
         				else {
         				sym.$("faux1").show();
         				}
         			if(input2.value==y)
         				{
         				sym.$("Vrai2").show();
         				correct=correct+0.5;
         				}
         				else {
         				sym.$("faux2").show();
         				}
         			if(input3.value==z)
         				{
         				sym.$("Vrai3").show();
         				correct=correct+0.5;
         				}
         				else
         				{
         				sym.$("faux3").show();
         				}
         			if(input4.value==w)
         				{
         				sym.$("Vrai4").show();
         				correct=correct+0.5;
         				}
         				else {
         				sym.$("faux4").show();
         				}
         				if(input5.value==b)
         				{
         			sym.$("Vrai5").show();
         			correct=correct+0.5;
         				}
         				else {
         				sym.$("faux5").show();
         				}
         				if(input6.value==c)
         				{
         				sym.$("Vrai6").show();
         				correct=correct+0.5;
         				}
         				else {
         				sym.$("faux6").show();
         				}
            if ((correct == 1)||(correct == 1.5))
            {
            sym.$('star_bot1').show();
            parent.Reveal.Coding[0]=1;
            }
            if ((correct == 2)||(correct == 2.5))
            {
            sym.$('star_bot1').show();
            sym.$('star_bot2').show();
            parent.Reveal.Coding[0]=2;
            }
            if (correct == 3)
            {
            sym.$('star_bot1').show();
            sym.$('star_bot2').show();
            sym.$('star_bot3').show();
            parent.Reveal.Coding[0]=3;
            }
         	}
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

      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
         // insert code to be run when the composition is fully loaded here
         click=1;
         click1=1;
         click2=1;
         
         sym.$("q2").hide();
         sym.$("q3").hide();
         sym.$("Alert").hide();
         sym.$("star_bot1").hide();
         sym.$("star_bot2").hide();
         sym.$("star_bot3").hide();
         for(j=1;j<7;j++)
         {
         sym.$("Vrai"+j).hide();
         sym.$("faux"+j).hide();
         
         }
         
         sym.$('Comp').html("<input type='number'  id='input1'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='border:0px ;background-color:transparent; width:40px; height:35px; color: Blue; padding-left:10px; text-align:left; font-size:30px;font-family: Trebuchet MS';>");
         sym.$('Intensit').html("<input type='number'  id='input2'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='border:0px ;background-color:transparent; width:60px; height:35px; color: Blue; padding-left:5px; text-align:left; font-size:30px;font-family: Trebuchet MS';'>");
         sym.$('PuisMoto1').html("<input type='number'  id='input3'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='border:0px ;background-color:transparent; width:60px; height:35px; color: Blue; padding-left:5px; text-align:left; font-size:30px;font-family: Trebuchet MS';'>");
         sym.$('PuisMoto2').html("<input type='number'  id='input4'" + "dir='ltr' tabindex='1' " +
         " style='border:0px ;background-color:transparent; width:60px; height:35px; color: Blue; padding-left:5px; text-align:left; font-size:30px;font-family: Trebuchet MS';'>");
         sym.$('Couleur1').html("<input type='number'  id='input5'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='border:0px ;background-color:transparent; width:40px; height:35px; color: Blue; padding-left:8px; text-align:left; font-size:30px;font-family: Trebuchet MS';'>");
         sym.$('Couleur2').html("<input type='number'  id='input6'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='border:0px ;background-color:transparent; width:40px; height:35px; color: Blue; padding-left:8px; text-align:left; font-size:30px;font-family: Trebuchet MS';'>");

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Indice2}", "click", function(sym, e) {
         // insert code for mouse click here
         if (click % 2)
         {
         sym.$("q3").show();
         click++;
         }
         else{
         sym.$("q3").hide();
         click++;
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Indice1}", "click", function(sym, e) {
         // insert code for mouse click here
         if (click1 % 2)
         {
         sym.$("q2").show();
         click1++;
         }
         else{
         sym.$("q2").hide();
         click1++;
         }

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Enonce}", "click", function(sym, e) {
         
         // insert code for mouse click here
         click2++;
         if(click2 % 2 == 0)
         {
         sym.$("Group").fadeOut(300);
         }
         else
         {
         sym.$("Group").fadeIn(300);
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