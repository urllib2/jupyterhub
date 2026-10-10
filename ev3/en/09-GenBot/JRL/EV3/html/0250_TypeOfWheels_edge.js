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
                            id: 'TITRE',
                            symbolName: 'TITRE',
                            type: 'rect',
                            rect: ['497px', '30px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'Lego-wheels-that-include-hubs-with-peg-holes',
                            type: 'image',
                            rect: ['561px', '279px', '530px', '393px', 'auto', 'auto'],
                            clip: 'rect(-41.666748046875px 590px 430px -68.75px)',
                            fill: ["rgba(0,0,0,0)",im+"A5_wheels.png",'0px','0px'],
                            transform: [[],[],[],['0.86915','0.86973']]
                        },
                        {
                            id: 'car1',
                            type: 'image',
                            rect: ['516px', '663px', '200px', '150px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a5_basic.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'car',
                            type: 'image',
                            rect: ['516px', '738px', '200px', '150px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"a5_basic.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'domestic1',
                            type: 'image',
                            rect: ['1116px', '663px', '200px', '150px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"A5_grande.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'domestic',
                            type: 'image',
                            rect: ['1116px', '738px', '200px', '150px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"A5_grande.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'arm1',
                            type: 'image',
                            rect: ['315px', '663px', '200px', '150px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a5_Petite.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'arm',
                            type: 'image',
                            rect: ['315px', '738px', '200px', '150px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"a5_Petite.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'tractor1',
                            type: 'image',
                            rect: ['716px', '663px', '200px', '150px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"A5_competition.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'tractor',
                            type: 'image',
                            rect: ['716px', '738px', '200px', '150px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"A5_competition.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'research1',
                            type: 'image',
                            rect: ['916px', '663px', '200px', '150px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"A5_geante.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'research',
                            type: 'image',
                            rect: ['916px', '738px', '200px', '150px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"A5_geante.png",'0px','0px','101.1%','101.1%', 'no-repeat']
                        },
                        {
                            id: 'rouePetite',
                            type: 'image',
                            rect: ['558px', '163px', '579px', '424px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a5_rouePetite.png",'0px','0px']
                        },
                        {
                            id: 'roueBasique',
                            type: 'image',
                            rect: ['478px', '105px', '710px', '520px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a5_roueBasique.png",'0px','0px']
                        },
                        {
                            id: 'roueCompetition',
                            type: 'image',
                            rect: ['508px', '114px', '680px', '510px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a5_roueCompetition.png",'0px','0px']
                        },
                        {
                            id: 'roueGrande',
                            type: 'image',
                            rect: ['474px', '-16px', '937px', '702px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"a5_roueGrande.png",'0px','0px']
                        },
                        {
                            id: 'petiteg',
                            type: 'group',
                            rect: ['329px', '239', '201', '207', 'auto', 'auto'],
                            c: [
                            {
                                id: 'petite1',
                                type: 'text',
                                rect: ['0px', '0px', '201px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">Small wheels</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [52, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'petite',
                                type: 'text',
                                rect: ['0px', '142px', '201px', '65px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">43.2x22mm</p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text5',
                                type: 'text',
                                rect: ['0px', '261px', '225px', '56px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​13.57cm</p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'basicg',
                            type: 'group',
                            rect: ['329px', '239', '225', '199', 'auto', 'auto'],
                            c: [
                            {
                                id: 'basic1',
                                type: 'text',
                                rect: ['0px', '0px', '249px', '78px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 32px;\">Basic wheels</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [52, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'basic',
                                type: 'text',
                                rect: ['0px', '142px', '212px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​56x28mm</p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text6',
                                type: 'text',
                                rect: ['0px', '262px', '201px', '30px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​17.59cm</p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'vpotureg',
                            type: 'group',
                            rect: ['329px', '239', '234', '182', 'auto', 'auto'],
                            c: [
                            {
                                id: 'voiture2',
                                type: 'text',
                                rect: ['0px', '0px', '234px', '65px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 36px;\">Car wheels</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [52, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'voiture',
                                type: 'text',
                                rect: ['0px', '125px', '234px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 32px;\">62.4x20mm</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [52, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text8',
                                type: 'text',
                                rect: ['0px', '260px', '180px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">19.59cm​</p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'motog',
                            type: 'group',
                            rect: ['329px', '260', '201', '169', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Text10',
                                type: 'text',
                                rect: ['0px', '0px', '259px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​Motorcycle wheel</p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text9',
                                type: 'text',
                                rect: ['0px', '104px', '201px', '65px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 32px;\">94.2x​20mm</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [52, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text14',
                                type: 'text',
                                rect: ['0px', '238px', '212px', '56px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">29.57cm</p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'grandeg',
                            type: 'group',
                            rect: ['329px', '239', '225', '189', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Text12',
                                type: 'text',
                                rect: ['0px', '124px', '200px', '65px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 32px;\">68.8x36mm</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [52, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text13',
                                type: 'text',
                                rect: ['0px', '0px', '225px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 32px;\">The Big wheel</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [52, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text11',
                                type: 'text',
                                rect: ['1px', '260px', '180px', '56px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​21.60cm</p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'dimensions',
                            type: 'group',
                            rect: ['329px', '176px', '216px', '190', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['0px', '0px', '201px', '65px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 40px; font-weight: 700; color: rgb(241, 158, 0);\">Type :</span></p>",
                                font: ['Arial, Helvetica, sans-serif', [52, "px"], "rgba(0,0,0,1)", "800", "none", "", "break-word", "normal"]
                            },
                            {
                                id: 'Text4',
                                type: 'text',
                                rect: ['0px', '253px', '316px', '56px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 40px; font-weight: 700; color: rgb(241, 158, 0);\">Circumference:</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [52, "px"], "rgba(0,0,0,1)", "800", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text3',
                                type: 'text',
                                rect: ['0px', '133px', '247px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 40px; font-weight: 700; color: rgb(241, 158, 0);\">Dimensions:</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [52, "px"], "rgba(0,0,0,1)", "800", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'head',
                            type: 'group',
                            rect: ['312px', '122', '1028', '138', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['92px', '0px', '869px', '158px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'RobotW',
                                type: 'image',
                                rect: ['-47px', '-1px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                            },
                            {
                                id: 'Text2',
                                type: 'text',
                                rect: ['140px', '31px', '764px', '78px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 36px; font-weight: 500;\">There are several types of wheels compatible with the EV3 kit.​</span></p>",
                                font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
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
                    duration: 5250,
                    autoPlay: false,
                    data: [

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
                            rect: ['62px', '0px', '655px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-size: 49px;\"><span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">types of wheels</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-51px', '0px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px;\">​<span style=\"font-family: \'Digital tech\'; font-weight: 500;\">types of wheels</span></p>',
                            font: ['Digital-7', [62, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 636px 57px 0px)',
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
                            [0,636,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0250_TypeOfWheels_edgeActions.js");
})("EDGE-6689876");
