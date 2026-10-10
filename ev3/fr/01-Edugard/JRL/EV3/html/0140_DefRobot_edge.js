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
                            rect: ['76px', '-4px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'RobotR',
                            type: 'image',
                            rect: ['312px', '132px', '160px', '160px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                        },
                        {
                            id: 'CASE_DEFINITION_GREEN2',
                            type: 'image',
                            rect: ['464px', '134px', '873px', '176px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                        },
                        {
                            id: 'Text2',
                            type: 'text',
                            rect: ['502px', '158px', '809px', '99px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">C'est un dispositif électromécanique qui est capable de réagir à son environnement et d'exécuter des actions autonomes pour accomplir une tâche &nbsp; &nbsp;</span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 32px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\"></span></p>",
                            align: "left",
                            font: ['Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE',
                            type: 'rect',
                            rect: ['178px', '32px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'a1_chien',
                            type: 'image',
                            rect: ['-89px', '269px', '800px', '600px', 'auto', 'auto'],
                            clip: 'rect(97.77783203125px 704.44482421875px 604.8961791992188px 196.45611572265625px)',
                            fill: ["rgba(0,0,0,0)",im+"a1_chien.jpg",'0px','0px'],
                            transform: [[],[],[],['0.88214','0.75585']]
                        },
                        {
                            id: 'a1_hum',
                            type: 'image',
                            rect: ['891px', '395px', '379px', '380px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a1_hum.jpg",'0px','0px']
                        },
                        {
                            id: 'a1_br',
                            type: 'image',
                            rect: ['1144px', '429px', '379px', '379px', 'auto', 'auto'],
                            clip: 'rect(0px 332.2424621582031px 379px 38.30078125px)',
                            fill: ["rgba(0,0,0,0)",im+"a1_br.jpg",'0px','0px'],
                            transform: [[],[],[],['1.10882']]
                        },
                        {
                            id: 'a1_cleaner',
                            type: 'image',
                            rect: ['482px', '417px', '390px', '341px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a1_cleaner.png",'0px','0px']
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
                    duration: 6500,
                    autoPlay: false,
                    data: [
                        [
                            "eid197",
                            "top",
                            4584,
                            0,
                            "linear",
                            "${TITRE}",
                            '32px',
                            '32px'
                        ],
                        [
                            "eid188",
                            "left",
                            4584,
                            0,
                            "linear",
                            "${TITRE}",
                            '178px',
                            '178px'
                        ]
                    ]
                }
            },
            "TITRE": {
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
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['337px', '-13px', '606px', '63px', 'auto', 'auto'],
                            id: 'Text6Copy2',
                            clip: 'rect(0px 627.099853515625px 65.823486328125px 0px)',
                            text: '<p style=\"margin: 0px; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\';\"><span style=\"font-weight: 500;\">definition D</span><span style=\"font-weight: 500; font-family: MagistralC;\">\'</span><span style=\"font-weight: 500;\">un robot</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['336px', '-14px', '636px', '62px', 'auto', 'auto'],
                            id: 'Text6',
                            clip: 'rect(0px 0px 62.88232421875px 0px)',
                            text: '<p style=\"margin: 0px; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; font-family: \'Digital tech\';\"><span style=\"font-weight: 500;\">definition d</span><span style=\"font-weight: 500; font-family: MagistralC;\">\'</span><span style=\"font-weight: 500;\">un robot</span></p>'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 4584,
                    autoPlay: true,
                    data: [
                        [
                            "eid195",
                            "height",
                            4584,
                            0,
                            "linear",
                            "${Text6}",
                            '62px',
                            '62px'
                        ],
                        [
                            "eid196",
                            "left",
                            4584,
                            0,
                            "linear",
                            "${Text6}",
                            '336px',
                            '336px'
                        ],
                        [
                            "eid132",
                            "clip",
                            0,
                            500,
                            "linear",
                            "${Text6}",
                            [0,0,62.88232421875,0],
                            [0,60,63.862548828125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid112",
                            "clip",
                            500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,60,63.862548828125,0],
                            [0,100,61.902099609375,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid114",
                            "clip",
                            750,
                            166,
                            "linear",
                            "${Text6}",
                            [0,100,61.902099609375,0],
                            [0,126.52281951904297,63.86279296875,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid198",
                            "clip",
                            916,
                            84,
                            "linear",
                            "${Text6}",
                            [0,126.52281951904297,63.86279296875,0],
                            [0,140,64.843017578125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid116",
                            "clip",
                            1000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,140,64.843017578125,0],
                            [0,160,66.803955078125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid118",
                            "clip",
                            1250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,160,66.803955078125,0],
                            [0,190,65.823486328125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid120",
                            "clip",
                            1500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,190,65.823486328125,0],
                            [0,220,65.823486328125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid122",
                            "clip",
                            1750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,220,65.823486328125,0],
                            [0,260,65.823486328125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid124",
                            "clip",
                            2000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,260,65.823486328125,0],
                            [0,270,64.843017578125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid126",
                            "clip",
                            2250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,270,64.843017578125,0],
                            [0,290,62.88232421875,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid128",
                            "clip",
                            2500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,290,62.88232421875,0],
                            [0,320,64.84326171875,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid130",
                            "clip",
                            2750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,320,64.84326171875,0],
                            [0,350,63.86279296875,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid157",
                            "clip",
                            3000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,350,63.86279296875,0],
                            [0,380,62.88232421875,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid159",
                            "clip",
                            3250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,380,62.88232421875,0],
                            [0,405,65.823486328125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid161",
                            "clip",
                            3500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,405,65.823486328125,0],
                            [0,475,64.843017578125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid165",
                            "clip",
                            3750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,475,64.843017578125,0],
                            [0,505,65.823486328125,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,505,65.823486328125,0],
                            [0,615.78955078125,62.88232421875,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid194",
                            "clip",
                            4250,
                            334,
                            "linear",
                            "${Text6}",
                            [0,615.78955078125,62.88232421875,0],
                            [0,615.78955078125,67.7841796875,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid192",
                            "top",
                            4584,
                            0,
                            "linear",
                            "${Text6}",
                            '-14px',
                            '-14px'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0140_DefRobot_edgeActions.js");
})("EDGE-6689876");
