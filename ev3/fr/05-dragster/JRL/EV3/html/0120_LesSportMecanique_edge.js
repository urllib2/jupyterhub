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
                            id: 'box',
                            symbolName: 'box',
                            type: 'rect',
                            rect: ['137px', '235px', '880', '578', 'auto', 'auto'],
                            transform: [[],[],[],['1','0.98078']]
                        },
                        {
                            id: 'Description',
                            type: 'text',
                            rect: ['1067px', '362px', '413px', '484px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​</p>",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "", "break-word", "normal"]
                        },
                        {
                            id: 'Text',
                            type: 'text',
                            rect: ['1091px', '370px', '406px', '86px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\">Les sports mécaniques sont des sports avec des véhicules à moteur comme les voitures, les quads, et les motos,…&nbsp;</p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: start; text-indent: 0px; line-height: normal;\">Il existe de nombreuses disciplines de sport mécanique</p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", ""],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Titre',
                            type: 'text',
                            rect: ['1067px', '256px', '428px', '97px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">Les Sports Mécanique​</p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [40, "px"], "rgba(28,0,184,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Next',
                            type: 'image',
                            rect: ['932px', '471px', '165px', '175px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"Next.png",'0px','0px'],
                            transform: [[],['180']]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_2_1',
                            type: 'rect',
                            rect: ['516px', '38px', '390', '57', 'auto', 'auto'],
                            transform: [[],[],[],['1.03999']]
                        },
                        {
                            id: 'pointer',
                            type: 'image',
                            rect: ['986px', '604px', '47px', '57px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"pointer.png",'0px','0px']
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
                    duration: 6000,
                    autoPlay: false,
                    data: [
                        [
                            "eid381",
                            "left",
                            0,
                            2000,
                            "easeInOutBack",
                            "${Titre}",
                            '175px',
                            '1058px'
                        ],
                        [
                            "eid382",
                            "left",
                            2000,
                            2000,
                            "easeInOutBack",
                            "${Description}",
                            '175px',
                            '1067px'
                        ],
                        [
                            "eid383",
                            "left",
                            4000,
                            2000,
                            "easeInOutBack",
                            "${Description}",
                            '175px',
                            '1079px'
                        ]
                    ]
                }
            },
            "TITRE_2_1": {
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
                            rect: ['-146px', '-12px', '872px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; text-transform: uppercase;\"><span style=\"text-transform: capitalize; font-size: 62px; font-weight: 500;\">les Sports mecanique</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 789.948486328125px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-21px', '-12px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; font-family: \'Digital tech\'; text-transform: uppercase;\"><span style=\"text-transform: capitalize; font-size: 62px; font-weight: 500;\">Les sports mecanique</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            id: 'Text6'
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
                            [0,661.60986328125,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            },
            "box": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            rect: ['0px', '5px', '880px', '572px', 'auto', 'auto'],
                            id: 'F31',
                            type: 'image',
                            fill: ['rgba(0,0,0,0)', 'images/F55.jpg', '0px', '0px']
                        },
                        {
                            rect: ['-892px', '5px', '880px', '572px', 'auto', 'auto'],
                            id: 'F6',
                            type: 'image',
                            fill: ['rgba(0,0,0,0)', 'images/F6.jpg', '0px', '0px']
                        },
                        {
                            type: 'image',
                            overflow: 'visible',
                            id: 'F22',
                            rect: ['-892px', '5px', '880px', '572px', 'auto', 'auto'],
                            fill: ['rgba(0,0,0,0)', 'images/F22.jpg', '0px', '0px']
                        },
                        {
                            rect: ['-892px', '5px', '880px', '572px', 'auto', 'auto'],
                            id: 'F32',
                            type: 'image',
                            fill: ['rgba(0,0,0,0)', 'images/F32.jpg', '0px', '0px']
                        },
                        {
                            rect: ['-892px', '5px', '880px', '572px', 'auto', 'auto'],
                            id: 'F12',
                            type: 'image',
                            fill: ['rgba(0,0,0,0)', 'images/F2.jpg', '0px', '0px']
                        },
                        {
                            rect: ['-892px', '5px', '880px', '572px', 'auto', 'auto'],
                            id: 'F42',
                            type: 'image',
                            fill: ['rgba(0,0,0,0)', 'images/F42.jpg', '0px', '0px']
                        },
                        {
                            rect: ['0px', '5px', '880px', '572px', 'auto', 'auto'],
                            id: 'F3',
                            type: 'image',
                            fill: ['rgba(0,0,0,0)', 'images/F55.jpg', '0px', '0px']
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            overflow: 'hidden',
                            rect: [null, null, '880px', '578px']
                        }
                    }
                },
                timeline: {
                    duration: 16250,
                    autoPlay: false,
                    data: [
                        [
                            "eid347",
                            "left",
                            3000,
                            1250,
                            "easeInOutBack",
                            "${F22}",
                            '-892px',
                            '0px'
                        ],
                        [
                            "eid379",
                            "left",
                            0,
                            1250,
                            "easeInOutBack",
                            "${F6}",
                            '-892px',
                            '0px'
                        ],
                        [
                            "eid348",
                            "left",
                            6000,
                            1250,
                            "easeInOutBack",
                            "${F32}",
                            '-892px',
                            '0px'
                        ],
                        [
                            "eid380",
                            "left",
                            15000,
                            1250,
                            "easeInOutBack",
                            "${F3}",
                            '-892px',
                            '0px'
                        ],
                        [
                            "eid350",
                            "left",
                            12000,
                            1250,
                            "easeInOutBack",
                            "${F42}",
                            '-892px',
                            '0px'
                        ],
                        [
                            "eid351",
                            "left",
                            9000,
                            1250,
                            "easeInOutBack",
                            "${F12}",
                            '-892px',
                            '0px'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0120_LesSportMecanique_edgeActions.js");
})("EDGE-6689876");
