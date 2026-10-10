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
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         // Masquer un élément 
         sym.$('Text1').html("<input type='text' id='input1' placeholder='....'" + " maxlength='100' dir='ltr' tabindex='1' " +
         " style='background-color:rgba(255,255,255,0.75); border:0px solid #124a7b; width:140px; height:67px; letter-spacing: 2px; color: red; padding-left:20px; text-align:left; font-size:70px;font-family: MagistralC;'>");
         
         sym.$("valider").hide();
         sym.$("play1").hide();
         sym.$("pause").hide();
         sym.$("ligne_1").hide();
         sym.$("Ex1_son").hide();
         play=1;
         hint=1;
         demo=1;
         
         sym.getSymbol("video").stop(0);
         
         
         // Masquer un élément 
         sym.$("Solution_Ex1").hide();
         
         
         
         // Masquer un élément 
         
         
         // Masquer un élément 
         sym.$("reflechir").hide();
         
         // Masquer un élément 
         sym.$("Rectangle2").hide();
         sym.$("solu").hide();
         
         // Masquer un élément 
         sym.$("solution").hide();
         
         
         // Masquer un élément 
         
         
         
         
         sym.play();
         
         

      });
      //Edge binding end

      

      

      Symbol.bindElementAction(compId, symbolName, "${Text32}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text32}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY2").hide();
         sym.$("ORANGE2").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text32}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY2").show();
         sym.$("ORANGE2").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY2}", "click", function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY2}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY2").hide();
         sym.$("ORANGE2").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${GREY2}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY2").show();
         sym.$("ORANGE2").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${solu}", "click", function(sym, e) {
         sym.$("solu").hide();
         sym.$("valider").fadeIn(500);
         sym.$("Ex1_son").fadeIn(500);
         sym.$("ligne_1").fadeIn(500);

      });
      //Edge binding end

      

      

      

      

      

      

      

      

            Symbol.bindElementAction(compId, symbolName, "${full_screen}", "click", function(sym, e) {
         sym.$("Text3").fadeOut(300);
         
         // insérer le code du clic de souris ici
         if (play==1) {
         
         sym.$('demo').fadeOut(700);
         sym.$('Hint').hide();
         sym.$('Reflexion').fadeOut(1500);
         sym.$('Intro').fadeOut(1500);
         sym.getSymbol("video").$("demo3")[0].currentTime=0;
         sym.getSymbol("video").$("demo3")[0].pause();
           sym.getSymbol("video").play(0);
           sym.$("full_screen").hide();
         
         setTimeout(function(){sym.$("full_screen").show();sym.$("full_screen").css({top: 420, left: 500 ,position:'absolute'}); sym.getSymbol("video").$("demo3")[0].play();   }, 1700);
         
          clearTimeout();
         
          play=-1;
         
         
         
         
                 sym.getSymbol("video").$("demo3")[0].ontimeupdate = function() {videoPause()};
                   function videoPause() {
         
                       if ((sym.getSymbol("video").$("demo3")[0].currentTime >=sym.getSymbol("video").$("demo3")[0].duration)&&((x>0)||(y>0))){
         
                           if (!sym.getSymbol("video").isPlaying()){
                             if (play!=1){
                                     sym.$("full_screen").hide();
                                     sym.getSymbol("video").playReverse(1500);
                                     play=1;
                                     sym.getSymbol("video").$("demo3")[0].currentTime=0;
                                     sym.getSymbol("video").$("demo3")[0].pause();
                                     sym.$('Reflexion').fadeIn(1500);
                            sym.$('Intro').fadeIn(1500);
                            sym.$("demo").fadeIn(1500)
                             sym.$("play1").show();
                              sym.$("pause").hide();
                              sym.$("demo").show();
                                     setTimeout(function(){ sym.$("demo").show(); sym.$("Hint").show();sym.$("full_screen").show(); sym.$("full_screen").css({top: 450, left: -106}); }, 1500);
                                     clearTimeout();
         
         
                               }
                              else
                                  {
         
                                  }
                                   }
         
                           else if (sym.getSymbol("video").isPlayingReverse()){
         
         
         
                             }
                            else if (sym.getSymbol("video").isPlaying()){
                                    sym.$("full_screen").hide();
                                    setTimeout(function(){ sym.getSymbol("video").playReverse(1500);  }, 500);
                                    clearTimeout();
                                    play=1;
                                    sym.getSymbol("video").$("demo3")[0].currentTime=0;
                                   sym.getSymbol("video").$("demo3")[0].pause();
                                   sym.$('Reflexion').fadeIn(1500);
                            sym.$('Intro').fadeIn(1500);
         
                                   setTimeout(function(){ sym.$("Hint").show();sym.$("demo").show(); sym.$("full_screen").show(); sym.$("full_screen").css({top: 450, left: -106}); }, 1500);
                                    clearTimeout();
         
         
                                     }
         
         
         
                         }
         
                 }
         
         
         }
         else 
         {
         if ((x!=0)||(y!=0)){
         play=1;
         sym.$("full_screen").hide();
         sym.getSymbol("video").playReverse(1500);
         sym.getSymbol("video").$("demo3")[0].currentTime=0;
         sym.getSymbol("video").$("demo3")[0].pause();
         sym.$('Reflexion').fadeIn(1500);
         sym.$('Intro').fadeIn(1500);
         
         setTimeout(function(){ sym.$("demo").show(); sym.$("Hint").show();sym.$("demo").show();sym.$("play1").show();sym.$("pause").hide(); sym.$("full_screen").show(); sym.$("full_screen").css({top: 450, left: -106}); }, 1500);
          clearTimeout();
         }
         
         
         };
         

      });
         //Edge binding end



      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 897, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2003, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         
         // insérer le code ici
         
         // insérer le code ici// insérer le code ici
         // Afficher un élément 
         sym.$("Compte_rebours").show();
         
         // Afficher un élément 
         sym.$("reflexion").show();
         
          y=3;
         x =0;
         sym.$("secondes").html("0"+x).css("color","#5abf6b");
         sym.$("minutes").html("0"+y).css("color","#5abf6b");
         
         
         var id = setInterval(frame, 1000);
         function frame() {
         if ((x<=1)&&(y==0)){
         
            clearInterval(id);
         
           sym.$("Reflexion").fadeOut(500);
           sym.$("solu").fadeIn(500); 
           sym.$("demonstration").fadeOut(500);
           sym.$("demo").hide();
           sym.$("Hint").hide();
         sym.$("Intro").fadeIn(500);
         
         } 
         
          if (y==0){
          x--;
          if (x<10) {
          sym.$("secondes").html("0"+x).css("color","#5abf6b");
          sym.$("minutes").html("0"+y).css("color","#5abf6b");
          }
          else
          {
           sym.$("secondes").html(x).css("color","#5abf6b");
         sym.$("minutes").html("0"+y).css("color","#5abf6b");
         
          }
          }
          else
          {
          if (x<=10){
         	if (x==0){
         	x=59;
         	y--;
         	sym.$("secondes").html(x).css("color","#5abf6b");
         	sym.$("minutes").html("0"+y).css("color","#5abf6b");
         
         	}
         	else{
         	x--;
         	sym.$("secondes").html("0"+x).css("color","#5abf6b");
         	sym.$("minutes").html("0"+y).css("color","#5abf6b");
              }
          }
         else {
         x--;
         sym.$("minutes").html("0"+y).css("color","#5abf6b");
         sym.$("secondes").html(x).css("color","#5abf6b");
         
         
         }	
         
          }
         
         
         
         
          }
         
         
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         // insérer le code ici
         
         // insérer le code ici// insérer le code ici
         // Afficher un élément 
         sym.$("Compte_rebours").show();
         
         // Afficher un élément 
         sym.$("reflexion").show();
         
         var y=0;
         var x = 10;
         sym.$("seconde").html(x).css("color","#5abf6b");
         sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         var x = 10;
         
         var id = setInterval(frame, 1000);
         function frame() {
         if ((x==0)&&(y==0)){
           clearInterval(id);
           sym.$("Reflexion").hide();
           sym.$("solu").show(); sym.$("demonstration").hide();
           sym.$("demo").hide();
           sym.$("Hint").hide();
         } 
         
          if (y==0){
          x--;
          if (x<10) {
          sym.$("seconde").html("0"+x).css("color","#5abf6b");
          sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
          }
          else
          {
           sym.$("seconde").html(x).css("color","#5abf6b");
         sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         
          }
          }
          else
          {
          if (x<=10){
          if (x==0){
          x=59;
          y--;
          sym.$("seconde").html(x).css("color","#5abf6b");
          sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         
          }
          else{
          x--;
          sym.$("seconde").html("0"+x).css("color","#5abf6b");
          sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
              }
          }
         else {
         x--;
         sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         sym.$("seconde").html(x).css("color","#5abf6b");
         
         
         }  
         
          }
         
         
         
         
          }
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "click", function(sym, e) {
         
         sym.$("Text3").hide();
         sym.$("pause").show();
         sym.$("demonstration").fadeIn(900);
         sym.$("full_screen").show();
         
         if (hint==-1){
         hint=1;
         sym.playReverse(2000);
         };
          sym.getSymbol("video").$("demo3")[0].ontimeupdate = function() {videoPause()};
         function videoPause() {
          if (sym.getSymbol("video").$("demo3")[0].currentTime >=sym.getSymbol("video").$("demo3")[0].duration){
          sym.$("play1").show();
           sym.$("pause").hide();
          }
         }
         
         
         if (sym.getSymbol("video").$("demo3")[0].paused) {
          sym.getSymbol("video").$("demo3")[0].play();
          sym.$("pause").show();
           sym.$("play1").hide();
         }
         else{
         sym.getSymbol("video").$("demo3")[0].pause();
         sym.$("play1").show();
          sym.$("pause").hide();
          }
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Text3}", "click", function(sym, e) {
         
         
         

      });
      //Edge binding end

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${indices}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         if (hint==1) {
         
         
         sym.play(1000);
         hint=-1;
         
         }
         else 
         {
         
         hint=1;
         sym.playReverse(2000);
         };

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "click", function(sym, e) {
         
         x=0;
         
         sym.$("message").show();
         sym.$('message').html('');
         
         
             if((input1.value>0)&&(input1.value<1000)){
         		sym.$('message').html('Valeurs enregistrées!');
         		input1.disabled = true;
         			sym.$("valider").unbind("click");
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
         sym.$("GREYCopy").hide();
         sym.$("ORANGECopy").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${valider}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREYCopy").show();
         sym.$("ORANGECopy").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "document", "keyup", function(sym, e) {
         if (e.which == 13) {
         sym.$("valider").click();
         	// exécuter une action
         }

      });
      //Edge binding end

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'video'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1500, function(sym, e) {
         // insérer le code ici
         sym.stop()

      });
      //Edge binding end

   })("video");
   //Edge symbol end:'video'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

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

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");