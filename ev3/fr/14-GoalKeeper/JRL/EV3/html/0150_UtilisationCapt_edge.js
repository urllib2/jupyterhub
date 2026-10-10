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
                scaleToFit: "none",
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
                            id: 'RoundRect',
                            type: 'rect',
                            rect: ['260px', '145px', '1072px', '677px', 'auto', 'auto'],
                            borderRadius: ["10px", "10px", "10px", "10px"],
                            fill: ["rgba(21,21,21,1.00)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE',
                            type: 'rect',
                            rect: ['531px', '29', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'Description',
                            type: 'text',
                            rect: ['329px', '835px', '989px', '199px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 34px; font-weight: 500;\"></span></p>",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "", "break-word", "normal"]
                        },
                        {
                            id: 'Description2',
                            type: 'text',
                            rect: ['338px', '835px', '961px', '199px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">Détecter si un mécanisme dans le robot a atteint une certaine position.​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 34px; font-weight: 500;\"></span></p>",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "", "break-word", "normal"]
                        },
                        {
                            id: 'Box',
                            symbolName: 'Box',
                            type: 'rect',
                            rect: ['275px', '157px', '1043', '654', 'auto', 'auto']
                        },
                        {
                            id: 'Symbol_1',
                            symbolName: 'Symbol_1',
                            type: 'rect',
                            rect: ['1318px', '701px', '203', '204', 'auto', 'auto']
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
                            "eid278",
                            "top",
                            0,
                            0,
                            "linear",
                            "${Box}",
                            '157px',
                            '157px'
                        ],
                        [
                            "eid275",
                            "left",
                            0,
                            0,
                            "linear",
                            "${Symbol_1}",
                            '1318px',
                            '1318px'
                        ],
                        [
                            "eid277",
                            "left",
                            0,
                            0,
                            "linear",
                            "${Box}",
                            '275px',
                            '275px'
                        ],
                        [
                            "eid276",
                            "top",
                            0,
                            0,
                            "linear",
                            "${Symbol_1}",
                            '701px',
                            '701px'
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
                            rect: ['-31px', '0px', '655px', '57px', 'auto', 'auto'],
                            id: 'Text6Copy2',
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-size: 49px;\"><span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">exemple d</span><span style=\"font-size: 62px; font-weight: 500;\">\'</span><span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">utilisation</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [62, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            rect: ['-82px', '0px', '699px', '57px', 'auto', 'auto'],
                            id: 'Text6',
                            clip: 'rect(0px 636px 57px 0px)',
                            text: '<p style=\"margin: 0px;\">​<span style=\"font-family: \'Digital tech\'; font-weight: 500;\">exemple d</span><span style=\"font-weight: 500;\">\'</span><span style=\"font-family: \'Digital tech\'; font-weight: 500;\">utilisation</span></p>'
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
                            [0,688.5,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            },
            "Box": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            type: 'image',
                            id: 'a10_gif32Copy',
                            rect: ['0px', '1px', '1043px', '690px', 'auto', 'auto'],
                            fill: ['rgba(0,0,0,0)', 'images/a10_gif3.gif', '0px', '0px']
                        },
                        {
                            type: 'image',
                            id: 'a10_gif22',
                            rect: ['-1px', '1px', '1043px', '654px', 'auto', 'auto'],
                            fill: ['rgba(0,0,0,0)', 'images/a10_gif2.gif', '0px', '0px']
                        },
                        {
                            type: 'image',
                            id: 'a10_gif12',
                            rect: ['0px', '7px', '1043px', '653px', 'auto', 'auto'],
                            fill: ['rgba(0,0,0,0)', 'images/a10_gif1.gif', '0px', '0px']
                        },
                        {
                            type: 'image',
                            id: 'a10_gif32',
                            rect: ['0px', '1px', '1043px', '690px', 'auto', 'auto'],
                            fill: ['rgba(0,0,0,0)', 'images/a10_gif3.gif', '0px', '0px']
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            overflow: 'hidden',
                            rect: [null, null, '1043px', '654px']
                        }
                    }
                },
                timeline: {
                    duration: 7500,
                    autoPlay: false,
                    data: [
                        [
                            "eid244",
                            "left",
                            3000,
                            1500,
                            "linear",
                            "${a10_gif12}",
                            '-1042px',
                            '0px'
                        ],
                        [
                            "eid247",
                            "top",
                            6000,
                            750,
                            "easeInElastic",
                            "${a10_gif32}",
                            '1px',
                            '2px'
                        ],
                        [
                            "eid248",
                            "top",
                            6750,
                            750,
                            "easeInElastic",
                            "${a10_gif32}",
                            '2px',
                            '1px'
                        ],
                        [
                            "eid239",
                            "left",
                            0,
                            1500,
                            "linear",
                            "${a10_gif22}",
                            '-1043px',
                            '-1px'
                        ],
                        [
                            "eid245",
                            "top",
                            3000,
                            1500,
                            "linear",
                            "${a10_gif12}",
                            '6px',
                            '1px'
                        ],
                        [
                            "eid246",
                            "left",
                            6000,
                            1500,
                            "linear",
                            "${a10_gif32}",
                            '-1042px',
                            '0px'
                        ],
                        [
                            "eid243",
                            "top",
                            0,
                            1500,
                            "linear",
                            "${a10_gif22}",
                            '0px',
                            '1px'
                        ]
                    ]
                }
            },
            "Next": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [

                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '203px', '204px']
                        }
                    }
                },
                timeline: {
                    duration: 0,
                    autoPlay: false,
                    data: [

                    ]
                }
            },
            "Symbol_1": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            transform: [[], ['180'], [0, 0, 0], [1, 1, 1]],
                            type: 'image',
                            id: 'Next',
                            rect: ['0px', '0px', '203px', '204px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ['rgba(0,0,0,0)', 'images/Next.png', '0px', '0px']
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '203px', '204px']
                        }
                    }
                },
                timeline: {
                    duration: 500,
                    autoPlay: false,
                    data: [
                        [
                            "eid269",
                            "scaleX",
                            0,
                            250,
                            "linear",
                            "${Next}",
                            '1',
                            '0.8'
                        ],
                        [
                            "eid271",
                            "scaleX",
                            250,
                            250,
                            "linear",
                            "${Next}",
                            '0.8',
                            '1'
                        ],
                        [
                            "eid270",
                            "scaleY",
                            0,
                            250,
                            "linear",
                            "${Next}",
                            '1',
                            '0.8'
                        ],
                        [
                            "eid272",
                            "scaleY",
                            250,
                            250,
                            "linear",
                            "${Next}",
                            '0.8',
                            '1'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0150_UtilisationCapt_edgeActions.js");
})("EDGE-6689876");
