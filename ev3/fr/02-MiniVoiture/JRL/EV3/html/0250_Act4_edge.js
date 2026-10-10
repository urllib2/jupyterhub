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
                centerStage: "horizontal",
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
                            id: 'Group',
                            type: 'group',
                            rect: ['157px', '592px', '1381', '289px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Question1',
                                type: 'group',
                                rect: ['170px', '37px', '340', '75', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'E_1',
                                    type: 'group',
                                    rect: ['444px', '108px', '80', '75', 'auto', 'auto'],
                                    c: [
                                    {
                                        id: 'CARRV1',
                                        type: 'image',
                                        rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                    },
                                    {
                                        id: 'CARRR1',
                                        type: 'image',
                                        rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                        clip: 'rect(0px 80px 86.111328125px 0px)',
                                        fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                    },
                                    {
                                        id: 'CARRG1',
                                        type: 'image',
                                        rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                    },
                                    {
                                        id: 'croix1',
                                        type: 'image',
                                        rect: ['17px', '12px', '46px', '46px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                    }]
                                },
                                {
                                    id: 'E_0',
                                    type: 'group',
                                    rect: ['130px', '108px', '80', '75', 'auto', 'auto'],
                                    c: [
                                    {
                                        id: 'CARRV0',
                                        type: 'image',
                                        rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                    },
                                    {
                                        id: 'CARRR0',
                                        type: 'image',
                                        rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                    },
                                    {
                                        id: 'CARRG0',
                                        type: 'image',
                                        rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                    },
                                    {
                                        id: 'croix0',
                                        type: 'image',
                                        rect: ['17px', '12px', '46px', '46px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                    }]
                                },
                                {
                                    id: 'Text3Copy',
                                    type: 'text',
                                    rect: ['125px', '190px', '376px', '51px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">&nbsp; &nbsp;A&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;<br></p><p style=\"margin: 0px;\">&nbsp; &nbsp; &nbsp;</p>",
                                    align: "left",
                                    font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1.00)", "900", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                },
                                {
                                    id: 'Text4',
                                    type: 'text',
                                    rect: ['744px', '191px', '222px', '94px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">&nbsp; &nbsp; &nbsp;C</p>",
                                    align: "left",
                                    font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1)", "900", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                },
                                {
                                    id: 'E_2',
                                    type: 'group',
                                    rect: ['778px', '108px', '80', '79px', 'auto', 'auto'],
                                    clip: 'rect(0px 80px 86.111083984375px 0px)',
                                    c: [
                                    {
                                        id: 'CARRV2',
                                        type: 'image',
                                        rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                        clip: 'rect(0px 80px 81.666748046875px 0px)',
                                        fill: ["rgba(0,0,0,0)",im+"CARR_VERT.png",'0px','0px']
                                    },
                                    {
                                        id: 'CARRR2',
                                        type: 'image',
                                        rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                        clip: 'rect(0px 80px 83.888916015625px 0px)',
                                        fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE.png",'0px','0px']
                                    },
                                    {
                                        id: 'CARRG2',
                                        type: 'image',
                                        rect: ['0px', '0px', '80px', '75px', 'auto', 'auto'],
                                        clip: 'rect(0px 80px 81.666748046875px 0px)',
                                        fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                    },
                                    {
                                        id: 'croix2',
                                        type: 'image',
                                        rect: ['17px', '12px', '46px', '46px', 'auto', 'auto'],
                                        fill: ["rgba(0,0,0,0)",im+"croix.png",'0px','0px']
                                    }]
                                }]
                            },
                            {
                                id: 'Text2',
                                type: 'text',
                                rect: ['606px', '221px', '74px', '48px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​ &nbsp; B</p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(205,135,0,1)", "900", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Attention',
                                type: 'text',
                                rect: ['478px', '253px', '441px', '29px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 40px;\">Cocher une reponse !</span></p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [37, "px"], "rgba(96,0,0,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'false2',
                                type: 'image',
                                rect: ['525px', '-12px', '300px', '140px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A2_choix2.png",'0px','0px']
                            },
                            {
                                id: 'false1',
                                type: 'image',
                                rect: ['194px', '-13px', '300px', '140px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A2_choix1.png",'0px','0px']
                            },
                            {
                                id: 'correct',
                                type: 'image',
                                rect: ['867px', '-16px', '300px', '140px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A2_choix3.png",'0px','0px']
                            },
                            {
                                id: 'bouton',
                                type: 'group',
                                rect: ['282px', '197px', '1040', '82', 'auto', 'auto'],
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
                                    id: 'Text322',
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
                                id: 'question',
                                type: 'group',
                                rect: ['296px', '-144px', '769', '120', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'Hint_green',
                                    type: 'image',
                                    rect: ['0px', '0px', '769px', '120px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"Hint_green.png",'0px','0px']
                                },
                                {
                                    id: 'Text6',
                                    type: 'text',
                                    rect: ['91px', '17px', '605px', '75px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 400;\">Cocher le programme qui donne le même résultat que celui &nbsp;ci-dessus</span></p>",
                                    align: "left",
                                    font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            },
                            {
                                id: 'Explication',
                                type: 'text',
                                rect: ['170px', '-118px', '1065px', '29px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: center;\">​On déduit que <span style=\"font-family: 'Times New Roman', Times, serif;\">&nbsp;: &nbsp;</span> <span style=\"font-size: 59px;\">&nbsp;</span><span style=\"font-size: 59px; font-weight: 900;\">1 rotation </span><span style=\"font-size: 59px; font-weight: 900; font-family: 'Times New Roman', Times, serif;\">= </span><span style=\"font-size: 59px; font-weight: 900;\">360° &nbsp;</span><span style=\"font-size: 45px; font-weight: 900;\"> &nbsp;</span></p>",
                                align: "left",
                                font: ['Lucida Sans Unicode, Lucida Grande, sans-serif', [40, "px"], "rgba(139,64,152,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Intro',
                            type: 'group',
                            rect: ['262px', '101px', '1128px', '177', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['121px', '22px', '900px', '250px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_VIOLET1.png",'0px','0px']
                            },
                            {
                                id: 'Text5',
                                type: 'text',
                                rect: ['157px', '-4px', '820px', '157px', 'auto', 'auto'],
                                text: "<p style=\"margin:0px\">​</p><p style=\"margin: 0px;\"><span style=\"font-weight: 800;\"></span></p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\">​</p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\">Tester le programme suivant :</p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\">​</p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\"></p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\">&nbsp;</p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\"><span style=\"font-weight: 800; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-style: normal; text-decoration: none; font-size: 34px; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">​</span></p><p style=\"margin:0px\">​</p><p style=\"margin:0px\">​</p>",
                                align: "left",
                                font: ['Tahoma, Geneva, sans-serif', [34, "px"], "rgba(0,0,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['166px', '149px', '800px', '47px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\">​​</p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">N'oubllier pas de placer la fleche à cette position</span></p>",
                                align: "left",
                                font: ['Tahoma, Geneva, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'A2_360deg',
                                type: 'image',
                                rect: ['625px', '46px', '350px', '137px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A2_360deg.png",'0px','0px']
                            },
                            {
                                id: 'ROBOT07',
                                type: 'image',
                                rect: ['-25px', '49px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotE.png",'0px','0px']
                            },
                            {
                                id: 'fleche',
                                type: 'image',
                                rect: ['365px', '-155px', '1024px', '768px', 'auto', 'auto'],
                                clip: 'rect(167.85693359375px 888.28564453125px 443px 685.714111328125px)',
                                fill: ["rgba(0,0,0,0)",im+"UntitledModel.png",'0px','0px'],
                                transform: [[],[],[],['0.29297','0.29297']]
                            }]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['692px', '30px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'demo',
                            type: 'group',
                            rect: ['-162px', '382px', '1051px', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1054.28564453125px 82px 900px)',
                            cursor: 'pointer',
                            transform: [[],[],[],['0.98953','1.09756']],
                            c: [
                            {
                                id: 'ORANGECopy',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREYCopy',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text3',
                                type: 'text',
                                rect: ['921px', '19px', '113px', '32px', 'auto', 'auto'],
                                clip: 'rect(5.479248046875px 116.58056640625px 45.478271484375px -2.97216796875px)',
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 26px;\">Question</span></p>",
                                align: "left",
                                font: ['MagistralC', [0, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
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
                    duration: 5000,
                    autoPlay: false,
                    data: [
                        [
                            "eid211",
                            "left",
                            2836,
                            0,
                            "linear",
                            "${TITRE}",
                            '692px',
                            '692px'
                        ]
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
                            font: ['Digital-7', [117, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6Copy2',
                            rect: ['-4px', '0px', '859px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 900px 57px -196.29614257813px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Activite</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [117, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6',
                            rect: ['-4px', '-1px', '766px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 650px 57px -155.55541992188px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">​<span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Activite</span></p>'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 5000,
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
                            [0,515,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,515,57,0],
                            [0,530,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid173",
                            "clip",
                            4250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,57,-155.555419921875],
                            [0,555,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid175",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,555,57,-155.555419921875],
                            [0,595,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid177",
                            "clip",
                            4750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,595,57,-155.555419921875],
                            [0,650,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid476",
                            "top",
                            5000,
                            0,
                            "linear",
                            "${Text6}",
                            '-1px',
                            '-1px'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0250_Act4_edgeActions.js");
})("EDGE-6689876");
