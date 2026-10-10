/*jslint */
/*global AdobeEdge: false, window: false, document: false, console:false, alert: false */
(function (compId) {

    "use strict";
    var im='images/',
        aud='media/',
        vid='media/',
        js='js/',
        fonts = {
            'Digital-7': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            '\'Digital tech\'': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            'MagistralC': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />'        },
        opts = {
            'gAudioPreloadPreference': 'auto',
            'gVideoPreloadPreference': 'auto'
        },
        resources = [
        ],
        scripts = [
            js+"jquery-2.0.3.min.js",
            js+"jquery.ui.touch-punch.min.js",
            js+"jquery-ui.min.js"
        ],
        symbols = {
            "stage": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "height",
                centerStage: "both",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['80px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'Text4',
                            type: 'text',
                            rect: ['295px', '294px', '1058px', '546px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; line-height: 46px;\"><span style=\"font-size: 36px; color: rgb(55, 0, 255); font-weight: 700;\">Saisir les valeurs qui manque dans les champs vides de programme à fin que le robot réalise la mission suivante:</span></p><p style=\"margin: 0px; line-height: 46px;\"><span style=\"font-size: 36px;\">​</span></p><p style=\"margin: 0px; line-height: 46px;\"><span style=\"font-size: 36px;\">​</span><span style=\"font-size: 36px; font-weight: 700; text-decoration: underline; color: rgb(2, 185, 14);\">si la valeur de lumiére détecté est supérieur à 25 :</span></p><p style=\"margin: 0px; line-height: 46px;\"><span style=\"font-size: 36px;\">-Le moteur branché au port D tourne dans </span><span style=\"font-size: 36px; font-weight: 700;\">le sens horraire</span><span style=\"font-size: 36px;\"> avec une puissance égale 50 puis clignte le temoin de la brique en </span><span style=\"font-size: 36px; font-weight: 700;\">vert pendant 3 secondes.</span></p><p style=\"margin: 0px; line-height: 46px;\"><span style=\"font-size: 36px;\">​</span><span style=\"font-size: 36px; font-weight: 700; text-decoration: underline; color: rgb(255, 0, 0);\">sinon&nbsp;</span></p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 400; font-style: normal; text-decoration: none; font-size: 24px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 46px;\"><span style=\"font-size: 36px;\">-Le moteur branché au port B tourne dans le sens </span><span style=\"font-size: 36px; font-weight: 700;\">anti-horraire</span><span style=\"font-size: 36px;\"> avec une puissance égale 50 puis clignte le temoin de la brique en </span><span style=\"font-size: 36px; font-weight: 700;\">rouge pendant 2 secondes.</span></p><p style=\"margin: 0px; line-height: 46px;\">​</p>",
                            align: "left",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Group',
                            type: 'group',
                            rect: ['155px', '117px', '1282px', '845px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'q1',
                                type: 'image',
                                rect: ['0px', '81px', '1257px', '694px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"q1%20%282%29.PNG",'0px','0px']
                            },
                            {
                                id: 'bouton',
                                type: 'group',
                                rect: ['125px', '747px', '1040', '82', 'auto', 'auto'],
                                clip: 'rect(0px 1040px 82px 900px)',
                                cursor: 'pointer',
                                c: [
                                {
                                    id: 'ORANGE',
                                    type: 'image',
                                    rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                                },
                                {
                                    id: 'GREY',
                                    type: 'image',
                                    rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                    cursor: 'pointer',
                                    fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                                },
                                {
                                    id: 'Text3',
                                    type: 'text',
                                    rect: ['916px', '17px', '111px', '44px', 'auto', 'auto'],
                                    cursor: 'pointer',
                                    text: "<p style=\"margin: 0px;\">​ &nbsp;Valider</p>",
                                    align: "left",
                                    font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            },
                            {
                                id: 'Alert',
                                type: 'text',
                                rect: ['183px', '745px', '584px', '54px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: center;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 35px; font-weight: 700; color: rgb(139, 0, 193);\">Vérifier votre réponse!!</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Comp',
                                type: 'text',
                                rect: ['301px', '456px', '41px', '37px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'Intensit',
                                type: 'text',
                                rect: ['350px', '456px', '41px', '36px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'PuisMoto2',
                                type: 'text',
                                rect: ['517px', '595px', '49px', '44px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'PuisMoto1',
                                type: 'text',
                                rect: ['525px', '322px', '41px', '36px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'Couleur1',
                                type: 'text',
                                rect: ['772px', '319px', '41px', '36px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'Couleur2',
                                type: 'text',
                                rect: ['772px', '595px', '41px', '36px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'q2',
                                type: 'image',
                                rect: ['0px', '77px', '1257px', '694px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"q2%20%282%29.PNG",'0px','0px']
                            },
                            {
                                id: 'q3',
                                type: 'image',
                                rect: ['0px', '81px', '1257px', '694px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"q3.PNG",'0px','0px']
                            },
                            {
                                id: 'pointer',
                                type: 'image',
                                rect: ['235px', '402px', '33px', '42px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"pointer.png",'0px','0px']
                            },
                            {
                                id: 'pointerCopy',
                                type: 'image',
                                rect: ['745px', '254px', '34px', '42px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"pointer.png",'0px','0px']
                            },
                            {
                                id: 'Indice1',
                                type: 'rect',
                                rect: ['222px', '387px', '93px', '69px', 'auto', 'auto'],
                                fill: ["rgba(255,255,255,0.00)"],
                                stroke: [4,"rgba(4, 169, 200, 0.639216)","none"]
                            },
                            {
                                id: 'Indice2',
                                type: 'rect',
                                rect: ['688px', '241px', '98px', '78px', 'auto', 'auto'],
                                fill: ["rgba(255,255,255,0.00)"],
                                stroke: [4,"rgba(4, 169, 200, 0.639216)","none"]
                            },
                            {
                                id: 'faux1',
                                type: 'image',
                                rect: ['258px', '296px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"faux.png",'0px','0px']
                            },
                            {
                                id: 'Vrai1',
                                type: 'image',
                                rect: ['255px', '294px', '76px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Vrai.png",'0px','0px']
                            },
                            {
                                id: 'faux2',
                                type: 'image',
                                rect: ['327px', '296px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"faux.png",'0px','0px']
                            },
                            {
                                id: 'Vrai2',
                                type: 'image',
                                rect: ['324px', '294px', '76px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Vrai.png",'0px','0px']
                            },
                            {
                                id: 'faux3',
                                type: 'image',
                                rect: ['540px', '158px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"faux.png",'0px','0px']
                            },
                            {
                                id: 'Vrai3',
                                type: 'image',
                                rect: ['537px', '156px', '76px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Vrai.png",'0px','0px']
                            },
                            {
                                id: 'faux4',
                                type: 'image',
                                rect: ['549px', '640px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"faux.png",'0px','0px']
                            },
                            {
                                id: 'Vrai4',
                                type: 'image',
                                rect: ['546px', '638px', '76px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Vrai.png",'0px','0px']
                            },
                            {
                                id: 'faux5',
                                type: 'image',
                                rect: ['765px', '156px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"faux.png",'0px','0px']
                            },
                            {
                                id: 'Vrai5',
                                type: 'image',
                                rect: ['762px', '156px', '76px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Vrai.png",'0px','0px']
                            },
                            {
                                id: 'faux6',
                                type: 'image',
                                rect: ['765px', '638px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"faux.png",'0px','0px']
                            },
                            {
                                id: 'Vrai6',
                                type: 'image',
                                rect: ['762px', '638px', '76px', '73px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Vrai.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'star_bot_op',
                            type: 'image',
                            rect: ['1211px', '106px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                        },
                        {
                            id: 'star_bot_op2',
                            type: 'image',
                            rect: ['1301px', '106px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                        },
                        {
                            id: 'star_bot_op3',
                            type: 'image',
                            rect: ['1386px', '106px', '64px', '73px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"star_bot_op.png",'0px','0px']
                        },
                        {
                            id: 'star_bot3',
                            type: 'image',
                            rect: ['1386px', '106px', '64px', '73px', 'auto', 'auto'],
                            opacity: '1',
                            fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                        },
                        {
                            id: 'star_bot2',
                            type: 'image',
                            rect: ['1301px', '106px', '64px', '73px', 'auto', 'auto'],
                            opacity: '1',
                            fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                        },
                        {
                            id: 'star_bot1',
                            type: 'image',
                            rect: ['1212px', '106px', '64px', '73px', 'auto', 'auto'],
                            opacity: '1',
                            fill: ["rgba(0,0,0,0)",im+"star_bot.png",'0px','0px']
                        },
                        {
                            id: 'ROBOT06',
                            type: 'image',
                            rect: ['168px', '107px', '160px', '160px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"RobotQ.png",'0px','0px']
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['531px', '29', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'cursorclick',
                            type: 'image',
                            rect: ['454px', '142px', '102px', '102px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"cursorclick.gif",'0px','0px']
                        },
                        {
                            id: 'Enonce',
                            type: 'text',
                            rect: ['357px', '106px', '289px', '70px', 'auto', 'auto'],
                            cursor: 'pointer',
                            text: "<p style=\"margin: 0px;\">​<span style=\"font-family: Georgia, 'Times New Roman', Times, serif; font-size: 55px; text-decoration: underline; color: rgb(255, 111, 0);\">Enoncé</span></p>",
                            align: "center",
                            font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                        }
                    ],
                    style: {
                        '${Stage}': {
                            isStage: true,
                            rect: ['null', 'null', '1650px', '1080', 'auto', 'auto'],
                            overflow: 'hidden',
                            fill: ["rgba(240,240,240,0)"]
                        }
                    }
                },
                timeline: {
                    duration: 5250,
                    autoPlay: false,
                    data: [

                    ]
                }
            },
            "TITRE_1": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6Copy2',
                            rect: ['200px', '-11px', '699px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(0, 61, 65); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">Quiz</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6',
                            rect: ['200px', '-11px', '699px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">Quiz</span></p>'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 5250,
                    autoPlay: true,
                    data: [
                        [
                            "eid132",
                            "clip",
                            0,
                            500,
                            "linear",
                            "${Text6}",
                            [0,0,57,0],
                            [0,60,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid112",
                            "clip",
                            500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,60,57,0],
                            [0,100,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid114",
                            "clip",
                            750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,100,57,0],
                            [0,140,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid116",
                            "clip",
                            1000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,140,57,0],
                            [0,160,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid118",
                            "clip",
                            1250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,160,57,0],
                            [0,190,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid120",
                            "clip",
                            1500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,190,57,0],
                            [0,220,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid122",
                            "clip",
                            1750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,220,57,0],
                            [0,260,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid124",
                            "clip",
                            2000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,260,57,0],
                            [0,270,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid126",
                            "clip",
                            2250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,270,57,0],
                            [0,290,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid128",
                            "clip",
                            2500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,290,57,0],
                            [0,320,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid130",
                            "clip",
                            2750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,320,57,0],
                            [0,350,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid157",
                            "clip",
                            3000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,350,57,0],
                            [0,380,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid159",
                            "clip",
                            3250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,380,57,0],
                            [0,405,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid161",
                            "clip",
                            3500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,405,57,0],
                            [0,475,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid165",
                            "clip",
                            3750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,475,57,0],
                            [0,505,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,505,57,0],
                            [0,530,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid199",
                            "clip",
                            4250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,57,0],
                            [0,540,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid201",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,540,57,0],
                            [0,565,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid203",
                            "clip",
                            4750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,565,57,0],
                            [0,595,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid205",
                            "clip",
                            5000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,595,57,0],
                            [0,519.83837890625,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0100_Quiz_edgeActions.js");
})("EDGE-6689876");
